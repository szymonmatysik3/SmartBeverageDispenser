#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h" 
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lvgl.h"


// 1. KONFIGURACJA PINÓW

#define PIN_MOSI      23
#define PIN_CLK       18
#define PIN_MISO      19 
#define PIN_LCD_CS    5
#define PIN_LCD_DC    21
#define PIN_LCD_RST   22

// ENKODER
#define ENC_PIN_A     26 
#define ENC_PIN_B     25 
#define ENC_PIN_BTN   27 

#define HX711_DT      32
#define HX711_SCK     33

// BUZZER
#define PIN_BUZZER    13

// POMPY
#define PIN_PUMP1     4   
#define PIN_PUMP2     16  
#define PIN_PUMP3     17
#define PIN_PUMP4     14

// PWM CONFIG
#define PUMP_PWM_TIMER    LEDC_TIMER_0
#define PUMP_PWM_MODE     LEDC_LOW_SPEED_MODE
#define PUMP_PWM_RES      LEDC_TIMER_10_BIT 
#define PUMP_PWM_FREQ     5000              

#define PUMP1_CHANNEL     LEDC_CHANNEL_0
#define PUMP2_CHANNEL     LEDC_CHANNEL_1
#define PUMP3_CHANNEL     LEDC_CHANNEL_2
#define PUMP4_CHANNEL     LEDC_CHANNEL_3

// LOGIKA POMP
#define PWM_MAX_SPEED     1023  
#define PWM_SLOW_SPEED    450   // Stała wolna prędkość na końcówce
#define SLOW_ZONE_GRAMS   5.0   // Kiedy włączyć wolny bieg

#define FLOW_TIMEOUT_US   10000000  
#define MIN_FLOW_GRANT    1.0      

#define HOST_ID           SPI2_HOST 

// 2. DANE
#define MAX_DRINKS 10

typedef struct {
    char name[32];
    int pump1_percent; 
    int pump2_percent; 
    int pump3_percent;
    int pump4_percent;
    int total_ml;      
    bool active;       
} Drink;

Drink drinks[MAX_DRINKS];
int drinks_count = 0;

spi_device_handle_t spi_lcd;
static lv_group_t * g;
static lv_style_t style_focus;
static lv_style_t style_red_btn;
static lv_style_t style_green_btn;
static lv_style_t style_no_anim;
static lv_style_t style_spinbox;

static lv_obj_t * spinbox_p1 = NULL;
static lv_obj_t * spinbox_p2 = NULL;
static lv_obj_t * spinbox_p3 = NULL;
static lv_obj_t * spinbox_p4 = NULL;
static lv_obj_t * spinbox_total = NULL;

static lv_obj_t * pouring_label_info = NULL;
static lv_obj_t * pouring_bar = NULL;
static lv_obj_t * kb_label_info = NULL;
static lv_obj_t * cleaning_label = NULL;
static lv_obj_t * btn_manual_start = NULL; 

static int global_enc_diff = 0;
static bool global_btn_pressed = false;
static bool is_keyboard_mode = false; 

// Waga
static volatile float current_weight_grams = 0.0f;
static volatile long scale_offset = 0;
static float scale_calibration = 1000.0f; 
static portMUX_TYPE hx711_lock = portMUX_INITIALIZER_UNLOCKED;

// Logika nalewania
static lv_timer_t * pour_timer = NULL;
static int pour_state = 0; 
static int stable_counter = 0;
static float last_weight = 0.0f; 

// Cele
static float target_cumulative_p1 = 0;
static float target_cumulative_p2 = 0;
static float target_cumulative_p3 = 0;
static float target_cumulative_total = 0; 

// Ilości
static float amount_p1 = 0;
static float amount_p2 = 0;
static float amount_p3 = 0;
static float amount_p4 = 0;

static int current_drink_idx = 0;

// PWM
static int current_applied_duty = -1;

// Logika czyszczenia
static int cleaning_state = 0; 

// Watchdog
static int64_t last_flow_time = 0;
static float flow_ref_weight = 0;

void hx711_init(void);
void init_peripherals_pwm_buzzer(void);
void start_pump_simple(int pump_id);
void update_pump_speed_simple(int pump_id, float target, float current);
void stop_pump(int pump_id);
void stop_all_pumps(void);
void buzzer_set(bool on);
void hx711_task(void *pvParameters);
void hx711_tare_now(void);
void save_drinks_to_nvs(void);
void load_main_menu(void);
void load_drink_list(bool is_edit_mode);
void load_edit_screen(int drink_index);
void load_keyboard_screen(int drink_index);
void load_pouring_screen(int drink_index);
void load_cleaning_menu(void);
void load_cleaning_run_screen(int mode);

// 3. STEROWANIE POMPAMI I BUZZEREM
void init_peripherals_pwm_buzzer() {
    gpio_reset_pin(PIN_BUZZER);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PUMP_PWM_MODE,
        .timer_num        = PUMP_PWM_TIMER,
        .duty_resolution  = PUMP_PWM_RES,
        .freq_hz          = PUMP_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t c1 = { .speed_mode=PUMP_PWM_MODE, .channel=PUMP1_CHANNEL, .timer_sel=PUMP_PWM_TIMER, .intr_type=LEDC_INTR_DISABLE, .gpio_num=PIN_PUMP1, .duty=0, .hpoint=0 };
    ESP_ERROR_CHECK(ledc_channel_config(&c1));
    ledc_channel_config_t c2 = { .speed_mode=PUMP_PWM_MODE, .channel=PUMP2_CHANNEL, .timer_sel=PUMP_PWM_TIMER, .intr_type=LEDC_INTR_DISABLE, .gpio_num=PIN_PUMP2, .duty=0, .hpoint=0 };
    ESP_ERROR_CHECK(ledc_channel_config(&c2));
    ledc_channel_config_t c3 = { .speed_mode=PUMP_PWM_MODE, .channel=PUMP3_CHANNEL, .timer_sel=PUMP_PWM_TIMER, .intr_type=LEDC_INTR_DISABLE, .gpio_num=PIN_PUMP3, .duty=0, .hpoint=0 };
    ESP_ERROR_CHECK(ledc_channel_config(&c3));
    ledc_channel_config_t c4 = { .speed_mode=PUMP_PWM_MODE, .channel=PUMP4_CHANNEL, .timer_sel=PUMP_PWM_TIMER, .intr_type=LEDC_INTR_DISABLE, .gpio_num=PIN_PUMP4, .duty=0, .hpoint=0 };
    ESP_ERROR_CHECK(ledc_channel_config(&c4));

    ledc_fade_func_install(0);
}

void buzzer_set(bool on) {
    if (on) gpio_set_level(PIN_BUZZER, 1); else gpio_set_level(PIN_BUZZER, 0);
}

void start_pump_simple(int pump_id) {
    int ch;
    if (pump_id == 1) ch = PUMP1_CHANNEL;
    else if (pump_id == 2) ch = PUMP2_CHANNEL;
    else if (pump_id == 3) ch = PUMP3_CHANNEL;
    else ch = PUMP4_CHANNEL;

    ledc_set_duty(PUMP_PWM_MODE, ch, PWM_MAX_SPEED);
    ledc_update_duty(PUMP_PWM_MODE, ch);
    current_applied_duty = PWM_MAX_SPEED; 
}

// MAX -> 5g -> SLOW
void update_pump_speed_simple(int pump_id, float target, float current) {
    float remaining = target - current;
    int target_duty = PWM_MAX_SPEED;

    if (remaining < SLOW_ZONE_GRAMS) {
        target_duty = PWM_SLOW_SPEED;
    }

    if (target_duty != current_applied_duty) {
        int ch;
        if (pump_id == 1) ch = PUMP1_CHANNEL;
        else if (pump_id == 2) ch = PUMP2_CHANNEL;
        else if (pump_id == 3) ch = PUMP3_CHANNEL;
        else ch = PUMP4_CHANNEL;

        ledc_set_duty(PUMP_PWM_MODE, ch, target_duty);
        ledc_update_duty(PUMP_PWM_MODE, ch);
        current_applied_duty = target_duty;
    }
}

void stop_pump(int pump_id) {
    int ch;
    if (pump_id == 1) ch = PUMP1_CHANNEL;
    else if (pump_id == 2) ch = PUMP2_CHANNEL;
    else if (pump_id == 3) ch = PUMP3_CHANNEL;
    else ch = PUMP4_CHANNEL;

    ledc_set_duty(PUMP_PWM_MODE, ch, 0);
    ledc_update_duty(PUMP_PWM_MODE, ch);
}

void stop_all_pumps() {
    stop_pump(1); stop_pump(2); stop_pump(3); stop_pump(4);
    current_applied_duty = 0;
}

// WAGA
void hx711_init() {
    gpio_reset_pin(HX711_DT); gpio_set_direction(HX711_DT, GPIO_MODE_INPUT);
    gpio_reset_pin(HX711_SCK); gpio_set_direction(HX711_SCK, GPIO_MODE_OUTPUT); gpio_set_level(HX711_SCK, 0);
}

void hx711_task(void *pvParameters) {
    hx711_init();
    float filtered_val = 0;
    while(1) {
        int timeout = 100;
        while (gpio_get_level(HX711_DT) == 1) {
            vTaskDelay(pdMS_TO_TICKS(2)); 
            timeout--; if(timeout == 0) break;
        }
        if (timeout > 0) {
            long count = 0;
            taskENTER_CRITICAL(&hx711_lock);
            for (int i = 0; i < 24; i++) {
                gpio_set_level(HX711_SCK, 1); ets_delay_us(1);
                count = count << 1;
                gpio_set_level(HX711_SCK, 0); ets_delay_us(1);
                if (gpio_get_level(HX711_DT)) count++;
            }
            gpio_set_level(HX711_SCK, 1); ets_delay_us(1);
            gpio_set_level(HX711_SCK, 0); ets_delay_us(1);
            taskEXIT_CRITICAL(&hx711_lock);
            if (count & 0x800000) count |= 0xFF000000;
            float raw_grams = (float)(count - scale_offset) / scale_calibration;
            
            filtered_val = (filtered_val * 0.3f) + (raw_grams * 0.7f);
            current_weight_grams = filtered_val;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void hx711_tare_now() {
    long current_raw = (long)(current_weight_grams * scale_calibration) + scale_offset;
    scale_offset = current_raw;
}

// 4. PAMIĘĆ NVS
void save_drinks_to_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_blob(my_handle, "drinks_data", drinks, sizeof(drinks));
        nvs_commit(my_handle); nvs_close(my_handle);
    }
}
void load_drinks_from_nvs() {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        size_t required_size = sizeof(drinks);
        if (nvs_get_blob(my_handle, "drinks_data", drinks, &required_size) == ESP_OK) {
            nvs_close(my_handle); return;
        }
        nvs_close(my_handle);
    }
    for(int i=0; i<MAX_DRINKS; i++) drinks[i].active = false;
    strcpy(drinks[0].name, "Mocny Start"); drinks[0].pump1_percent=40; drinks[0].pump2_percent=60; drinks[0].pump3_percent=0; drinks[0].pump4_percent=0; drinks[0].total_ml=100; drinks[0].active=true;
    strcpy(drinks[1].name, "Turbo Mix");     drinks[1].pump1_percent=25; drinks[1].pump2_percent=25; drinks[1].pump3_percent=25; drinks[1].pump4_percent=25; drinks[1].total_ml=200; drinks[1].active=true;
    save_drinks_to_nvs();
}
int create_new_drink() {
    for(int i=0; i<MAX_DRINKS; i++) {
        if (!drinks[i].active) {
            drinks[i].active = true; strcpy(drinks[i].name, "Nowy"); 
            drinks[i].pump1_percent=25; drinks[i].pump2_percent=25; drinks[i].pump3_percent=25; drinks[i].pump4_percent=25;
            drinks[i].total_ml=100;
            save_drinks_to_nvs(); return i;
        }
    } return -1;
}
void delete_drink(int index) {
    if (index >= 0 && index < MAX_DRINKS) { drinks[index].active = false; save_drinks_to_nvs(); }
}

// 5. STEROWNIKI LCD
void lcd_send_cmd(const uint8_t cmd) {
    spi_transaction_t t; memset(&t, 0, sizeof(t)); t.length = 8; t.tx_buffer = &cmd;
    gpio_set_level(PIN_LCD_DC, 0); spi_device_polling_transmit(spi_lcd, &t);
}
void lcd_send_data(const uint8_t *data, int len) {
    if (len == 0) return;
    spi_transaction_t t; memset(&t, 0, sizeof(t)); t.length = len * 8; t.tx_buffer = data;
    gpio_set_level(PIN_LCD_DC, 1); spi_device_polling_transmit(spi_lcd, &t);
}
void ili9488_init() {
    gpio_set_level(PIN_LCD_RST, 0); vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(100));
    lcd_send_cmd(0xE0); uint8_t g1[] = {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}; lcd_send_data(g1, 15);
    lcd_send_cmd(0xE1); uint8_t g2[] = {0x00, 0x16, 0x19, 0x03, 0x11, 0x05, 0x35, 0x24, 0x53, 0x07, 0x08, 0x04, 0x13, 0x1A, 0x0F}; lcd_send_data(g2, 15);
    lcd_send_cmd(0xC0); uint8_t p1[] = {0x17, 0x15}; lcd_send_data(p1, 2);
    lcd_send_cmd(0xC1); uint8_t p2[] = {0x41}; lcd_send_data(p2, 1);
    lcd_send_cmd(0xC5); uint8_t vc[] = {0x00, 0x12, 0x80}; lcd_send_data(vc, 3);
    lcd_send_cmd(0x36); uint8_t mad[] = {0x28}; lcd_send_data(mad, 1); 
    lcd_send_cmd(0x3A); uint8_t fmt[] = {0x66}; lcd_send_data(fmt, 1); 
    lcd_send_cmd(0xB0); uint8_t imc[] = {0x00}; lcd_send_data(imc, 1);
    lcd_send_cmd(0xB1); uint8_t frc[] = {0xA0}; lcd_send_data(frc, 1);
    lcd_send_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));
    lcd_send_cmd(0x29); 
}
#define SPI_BLOCK_SIZE 512 
void disp_driver_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p) {
    uint8_t x_data[] = {(area->x1 >> 8), (area->x1 & 0xFF), (area->x2 >> 8), (area->x2 & 0xFF)};
    uint8_t y_data[] = {(area->y1 >> 8), (area->y1 & 0xFF), (area->y2 >> 8), (area->y2 & 0xFF)};
    lcd_send_cmd(0x2A); lcd_send_data(x_data, 4);
    lcd_send_cmd(0x2B); lcd_send_data(y_data, 4);
    lcd_send_cmd(0x2C); 
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    uint8_t *data_buf = heap_caps_malloc(SPI_BLOCK_SIZE * 3, MALLOC_CAP_DMA);
    if (!data_buf) { lv_disp_flush_ready(drv); return; }
    uint32_t sent_pixels = 0;
    gpio_set_level(PIN_LCD_DC, 1); 
    while (sent_pixels < size) {
        uint32_t pixels_to_send = size - sent_pixels;
        if (pixels_to_send > SPI_BLOCK_SIZE) pixels_to_send = SPI_BLOCK_SIZE;
        uint32_t buf_idx = 0;
        for (uint32_t i = 0; i < pixels_to_send; i++) {
            uint16_t pixel = color_p[sent_pixels + i].full;
            data_buf[buf_idx++] = (pixel & 0xF800) >> 8; 
            data_buf[buf_idx++] = (pixel & 0x07E0) >> 3; 
            data_buf[buf_idx++] = (pixel & 0x001F) << 3; 
        }
        spi_transaction_t t; memset(&t, 0, sizeof(t)); t.length = pixels_to_send * 3 * 8; t.tx_buffer = data_buf;
        spi_device_polling_transmit(spi_lcd, &t);
        sent_pixels += pixels_to_send;
    }
    free(data_buf);
    lv_disp_flush_ready(drv);
}
void lvgl_encoder_read(lv_indev_drv_t * drv, lv_indev_data_t * data) {
    data->enc_diff = global_enc_diff; global_enc_diff = 0; 
    if (global_btn_pressed) data->state = LV_INDEV_STATE_PRESSED; else data->state = LV_INDEV_STATE_RELEASED;
}
void clear_screen() {
    lv_group_remove_all_objs(g); lv_obj_clean(lv_scr_act());
    spinbox_p1 = NULL; spinbox_p2 = NULL; spinbox_p3 = NULL; spinbox_p4 = NULL; spinbox_total = NULL;
    pouring_label_info = NULL; pouring_bar = NULL; kb_label_info = NULL; cleaning_label = NULL;
    btn_manual_start = NULL; 
    is_keyboard_mode = false; buzzer_set(false);
}

// 6. LOGIKA GUI - NALEWANIE

void check_flow_and_speed(int pump_id, float target, float current) {
    int64_t now = esp_timer_get_time();
    if (current > flow_ref_weight + MIN_FLOW_GRANT) {
        flow_ref_weight = current; last_flow_time = now;
    }
    if ((now - last_flow_time) > FLOW_TIMEOUT_US) {
        stop_all_pumps(); pour_state = 99; buzzer_set(true); return;
    }
    update_pump_speed_simple(pump_id, target, current);
}

void pouring_timer_cb(lv_timer_t * timer) {
    if (!pouring_label_info) return;
    float current_weight = current_weight_grams; 
    int w_int = (int)current_weight; 
    
    if (pour_state >= 30 && pour_state <= 33) {
        if (btn_manual_start) lv_obj_add_flag(btn_manual_start, LV_OBJ_FLAG_HIDDEN);
        if (current_weight < -10.0) { stop_all_pumps(); pour_state = 0; lv_label_set_text(pouring_label_info, "SZKLO ZABRANE!\nSTOP"); return; }
    }

    if (pour_state == 99) {
        lv_label_set_text(pouring_label_info, "BLAD PRZEPLYWU!\nSPRAWDZ ZBIORNIKI");
        if ((stable_counter % 10) < 5) buzzer_set(true); else buzzer_set(false);
        stable_counter++;
        if(stable_counter > 50) { buzzer_set(false); if(pour_timer) lv_timer_del(pour_timer); load_drink_list(false); }
        return; 
    }

    if (pour_state == 0) { 
        lv_label_set_text_fmt(pouring_label_info, "POLOZ SZKLANKE\nLUB WCISNIJ START\nWaga: %d g", w_int);
        if (current_weight > 10.0) { pour_state = 1; stable_counter = 0; }
    }
    else if (pour_state == 1) { 
        if (btn_manual_start) lv_obj_add_flag(btn_manual_start, LV_OBJ_FLAG_HIDDEN);
        if (current_weight < 5.0) { pour_state = 0; stable_counter = 0; return; }
        lv_label_set_text_fmt(pouring_label_info, "STABILIZACJA...\nWaga: %d g", w_int);
        if (fabs(current_weight - last_weight) < 2.0) stable_counter++; else stable_counter = 0;
        last_weight = current_weight;
        if (stable_counter > 15) { hx711_tare_now(); pour_state = 2; stable_counter = 20; }
    }
    else if (pour_state == 2) { 
        if (btn_manual_start) lv_obj_add_flag(btn_manual_start, LV_OBJ_FLAG_HIDDEN);
        if (current_weight < -10.0) { pour_state = 0; stable_counter = 0; return; }
        int time_int = stable_counter / 10;
        lv_label_set_text_fmt(pouring_label_info, "START ZA: %d s\nWaga: %d g", time_int, w_int);
        stable_counter--;
        if (stable_counter <= 0) {
            float total = (float)drinks[current_drink_idx].total_ml;
            int s1 = drinks[current_drink_idx].pump1_percent;
            int s2 = drinks[current_drink_idx].pump2_percent;
            int s3 = drinks[current_drink_idx].pump3_percent;
            int s4 = drinks[current_drink_idx].pump4_percent;
            int sum = s1 + s2 + s3 + s4;
            if (sum == 0) sum = 1;

            amount_p1 = total * ((float)s1 / (float)sum);
            amount_p2 = total * ((float)s2 / (float)sum);
            amount_p3 = total * ((float)s3 / (float)sum);
            amount_p4 = total * ((float)s4 / (float)sum);

            target_cumulative_p1 = amount_p1;
            target_cumulative_p2 = amount_p1 + amount_p2;
            target_cumulative_p3 = amount_p1 + amount_p2 + amount_p3;
            target_cumulative_total = total;

            last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight;
            pour_state = 30; 
        }
    }
    // POMPY
    else if (pour_state == 30) {
        if (amount_p1 < 1.0) { last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight; pour_state = 31; return; }
        if (gpio_get_level(PIN_PUMP1) == 0) start_pump_simple(1);
        check_flow_and_speed(1, target_cumulative_p1, current_weight);
        if(pour_state == 99) return;
        lv_label_set_text_fmt(pouring_label_info, "POMPA 1...\n%d / %d ml", w_int, (int)target_cumulative_total);
        if(pouring_bar) lv_bar_set_value(pouring_bar, (int)(current_weight/target_cumulative_total*100), LV_ANIM_ON);
        if (current_weight >= target_cumulative_p1) {
            stop_pump(1); last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight; pour_state = 31;
        }
    }
    else if (pour_state == 31) {
        if (amount_p2 < 1.0) { last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight; pour_state = 32; return; }
        if (gpio_get_level(PIN_PUMP2) == 0) start_pump_simple(2);
        check_flow_and_speed(2, target_cumulative_p2, current_weight);
        if(pour_state == 99) return;
        lv_label_set_text_fmt(pouring_label_info, "POMPA 2...\n%d / %d ml", w_int, (int)target_cumulative_total);
        if(pouring_bar) lv_bar_set_value(pouring_bar, (int)(current_weight/target_cumulative_total*100), LV_ANIM_ON);
        if (current_weight >= target_cumulative_p2) {
            stop_pump(2); last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight; pour_state = 32;
        }
    }
    else if (pour_state == 32) {
        if (amount_p3 < 1.0) { last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight; pour_state = 33; return; }
        if (gpio_get_level(PIN_PUMP3) == 0) start_pump_simple(3);
        check_flow_and_speed(3, target_cumulative_p3, current_weight);
        if(pour_state == 99) return;
        lv_label_set_text_fmt(pouring_label_info, "POMPA 3...\n%d / %d ml", w_int, (int)target_cumulative_total);
        if(pouring_bar) lv_bar_set_value(pouring_bar, (int)(current_weight/target_cumulative_total*100), LV_ANIM_ON);
        if (current_weight >= target_cumulative_p3) {
            stop_pump(3); last_flow_time = esp_timer_get_time(); flow_ref_weight = current_weight; pour_state = 33;
        }
    }
    else if (pour_state == 33) {
        if (amount_p4 < 1.0) { stop_all_pumps(); pour_state = 4; return; }
        if (gpio_get_level(PIN_PUMP4) == 0) start_pump_simple(4);
        check_flow_and_speed(4, target_cumulative_total, current_weight);
        if(pour_state == 99) return;
        lv_label_set_text_fmt(pouring_label_info, "POMPA 4...\n%d / %d ml", w_int, (int)target_cumulative_total);
        if(pouring_bar) lv_bar_set_value(pouring_bar, (int)(current_weight/target_cumulative_total*100), LV_ANIM_ON);
        if (current_weight >= target_cumulative_total) { stop_pump(4); pour_state = 4; }
    }
    else if (pour_state == 4) { 
        stop_all_pumps();
        if(pouring_bar) lv_bar_set_value(pouring_bar, 100, LV_ANIM_ON);
        lv_label_set_text(pouring_label_info, "GOTOWE!\nZabierz szklo.");
        if (current_weight < -10.0) { pour_state = 5; stable_counter = 40; }
    }
    else if (pour_state == 5) { 
        int t = (stable_counter / 10) + 1;
        lv_label_set_text_fmt(pouring_label_info, "POWROT ZA %d s...", t);
        stable_counter--;
        if (stable_counter <= 0) { if (pour_timer) lv_timer_del(pour_timer); load_drink_list(false); }
    }
}

// RĘCZNY START
static void manual_start_cb(lv_event_t * e) {
    if (pour_state == 0 || pour_state == 1) {
        hx711_tare_now(); 
        pour_state = 2; stable_counter = 10; 
        if (btn_manual_start) lv_obj_add_flag(btn_manual_start, LV_OBJ_FLAG_HIDDEN);
    }
}

static void pour_back_cb(lv_event_t * e) {
    stop_all_pumps();
    if (pour_timer) { lv_timer_del(pour_timer); pour_timer = NULL; }
    load_drink_list(false);
}

void load_pouring_screen(int idx) {
    clear_screen(); hx711_tare_now(); pour_state = 0; stable_counter = 0; current_drink_idx = idx; 
    
    lv_obj_t * title = lv_label_create(lv_scr_act()); 
    lv_label_set_text_fmt(title, "Wybrano: %s", drinks[idx].name); lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    pouring_label_info = lv_label_create(lv_scr_act()); 
    lv_label_set_text(pouring_label_info, "TAROWANIE..."); 
    lv_obj_set_style_text_align(pouring_label_info, LV_TEXT_ALIGN_CENTER, 0); 
    lv_obj_align(pouring_label_info, LV_ALIGN_CENTER, 0, -30);
    
    pouring_bar = lv_bar_create(lv_scr_act()); 
    lv_obj_set_size(pouring_bar, 300, 20); lv_obj_align(pouring_bar, LV_ALIGN_CENTER, 0, 30); 
    lv_bar_set_range(pouring_bar, 0, 100); lv_bar_set_value(pouring_bar, 0, LV_ANIM_OFF);
    
    // START BUTTON
    btn_manual_start = lv_btn_create(lv_scr_act()); 
    lv_obj_set_size(btn_manual_start, 120, 50);
    lv_obj_align(btn_manual_start, LV_ALIGN_BOTTOM_LEFT, 20, -10); 
    lv_obj_add_style(btn_manual_start, &style_green_btn, 0); lv_obj_add_style(btn_manual_start, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(btn_manual_start, manual_start_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t * l_start = lv_label_create(btn_manual_start); lv_label_set_text(l_start, "START"); lv_obj_center(l_start);

    // CANCEL BUTTON
    lv_obj_t * btn_back = lv_btn_create(lv_scr_act()); 
    lv_obj_set_size(btn_back, 120, 50);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_RIGHT, -20, -10); 
    lv_obj_add_style(btn_back, &style_red_btn, 0);
    lv_obj_add_style(btn_back, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(btn_back, pour_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t * l_back = lv_label_create(btn_back); lv_label_set_text(l_back, "ANULUJ"); lv_obj_center(l_back);

    pour_timer = lv_timer_create(pouring_timer_cb, 100, NULL);
    
    lv_group_add_obj(g, btn_manual_start);
    lv_group_add_obj(g, btn_back);
    lv_group_focus_obj(btn_manual_start);
}

// 7. CZYSZCZENIE
static void cleaning_action_cb(lv_event_t * e) {
    if (cleaning_state == 5) { 
        stop_all_pumps();
        start_pump_simple(3); start_pump_simple(4);
        cleaning_state = 6;
        if (cleaning_label) { lv_label_set_text(cleaning_label, "CZYSZCZENIE...\nPOMPA 3 + 4\n[KLIKNIJ ABY ZAKONCZYC]"); }
    } else {
        stop_all_pumps();
        cleaning_state = 0;
        load_cleaning_menu();
    }
}

void load_cleaning_run_screen(int mode) {
    clear_screen();
    cleaning_state = mode;
    
    if (mode == 1) { start_pump_simple(1); }
    if (mode == 2) { start_pump_simple(2); }
    if (mode == 3) { start_pump_simple(3); }
    if (mode == 4) { start_pump_simple(4); }
    if (mode == 5) { start_pump_simple(1); start_pump_simple(2); }


    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 400, 200);
    lv_obj_center(btn);
    lv_obj_add_style(btn, &style_focus, LV_STATE_FOCUSED); 
    lv_group_add_obj(g, btn); 
    lv_obj_add_event_cb(btn, cleaning_action_cb, LV_EVENT_CLICKED, NULL);
    
    cleaning_label = lv_label_create(btn);
    if (mode == 5) { lv_label_set_text(cleaning_label, "CZYSZCZENIE...\nPOMPA 1 + 2\n[KLIKNIJ DALEJ]"); }
    else { lv_label_set_text_fmt(cleaning_label, "CZYSZCZENIE...\nPOMPA %d\n[KLIKNIJ STOP]", mode); }
    lv_obj_set_style_text_align(cleaning_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(cleaning_label);

    lv_group_focus_obj(btn);
}

static void clean_item_cb(lv_event_t * e) {
    int mode = (int)lv_event_get_user_data(e);
    load_cleaning_run_screen(mode);
}

void load_cleaning_menu() {
    clear_screen();
    lv_obj_t * list = lv_list_create(lv_scr_act());
    lv_obj_set_size(list, 400, 260); lv_obj_center(list);
    lv_list_add_text(list, "WYBIERZ POMPE");
    
    const char * names[] = {"POMPA 1", "POMPA 2", "POMPA 3", "POMPA 4", "WSZYSTKIE"};
    for(int i=0; i<5; i++) {
        lv_obj_t * btn = lv_list_add_btn(list, LV_SYMBOL_REFRESH, names[i]);
        lv_group_add_obj(g, btn); lv_obj_add_style(btn, &style_focus, LV_STATE_FOCUSED);
        lv_obj_add_event_cb(btn, clean_item_cb, LV_EVENT_CLICKED, (void*)(i+1));
        lv_obj_add_style(btn, &style_no_anim, 0);
    }
    
    lv_obj_t * b = lv_list_add_btn(list, LV_SYMBOL_LEFT, "WROC");
    lv_group_add_obj(g, b); lv_obj_add_style(b, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(b, (lv_event_cb_t)load_main_menu, LV_EVENT_CLICKED, NULL);
    lv_obj_add_style(b, &style_no_anim, 0);
}


static void save_btn_cb(lv_event_t * e) {
    int idx = (int)lv_event_get_user_data(e);
    if(spinbox_p1) drinks[idx].pump1_percent = lv_spinbox_get_value(spinbox_p1);
    if(spinbox_p2) drinks[idx].pump2_percent = lv_spinbox_get_value(spinbox_p2);
    if(spinbox_p3) drinks[idx].pump3_percent = lv_spinbox_get_value(spinbox_p3);
    if(spinbox_p4) drinks[idx].pump4_percent = lv_spinbox_get_value(spinbox_p4);
    if(spinbox_total) drinks[idx].total_ml = lv_spinbox_get_value(spinbox_total);
    save_drinks_to_nvs(); load_drink_list(true);
}
static void name_btn_cb(lv_event_t * e) { load_keyboard_screen((int)lv_event_get_user_data(e)); }
static void delete_btn_cb(lv_event_t * e) { delete_drink((int)lv_event_get_user_data(e)); load_drink_list(true); }

lv_obj_t* create_spinbox_cell(lv_obj_t * parent, const char* name, int val, int min, int max, bool is_ml) {
    lv_obj_t * cont = lv_obj_create(parent);
    lv_obj_set_size(cont, 230, 50);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(cont, 2, 0);

    // Etykieta
    lv_obj_t * l = lv_label_create(cont);
    lv_label_set_text(l, name);

    // Spinbox
    lv_obj_t * sb = lv_spinbox_create(cont);
    lv_spinbox_set_range(sb, min, max);
    lv_spinbox_set_digit_format(sb, 3, 0);
    lv_spinbox_set_step(sb, 1);
    lv_spinbox_set_value(sb, val);
    lv_obj_set_width(sb, 70);
    lv_obj_add_style(sb, &style_spinbox, 0);
    lv_obj_add_style(sb, &style_focus, LV_STATE_FOCUSED);
    lv_group_add_obj(g, sb);

    // Jednostka
    lv_obj_t * unit = lv_label_create(cont);
    lv_label_set_text(unit, is_ml ? "ml" : "%");

    return sb;
}

void load_edit_screen(int idx) {
    clear_screen();
    
    lv_obj_t * main_col = lv_obj_create(lv_scr_act());
    lv_obj_set_size(main_col, 480, 320); 
    lv_obj_center(main_col);
    lv_obj_set_layout(main_col, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(main_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_bg_opa(main_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(main_col, 0, 0);
    lv_obj_set_style_pad_gap(main_col, 5, 0);

    // 1. NAZWA
    lv_obj_t * name_btn = lv_btn_create(main_col);
    lv_obj_set_size(name_btn, 400, 35);
    lv_obj_add_flag(name_btn, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK); 
    lv_obj_add_style(name_btn, &style_focus, LV_STATE_FOCUSED);
    lv_group_add_obj(g, name_btn);
    lv_obj_add_event_cb(name_btn, name_btn_cb, LV_EVENT_CLICKED, (void*)idx);
    lv_obj_t * l_n = lv_label_create(name_btn);
    lv_label_set_text_fmt(l_n, "Nazwa: %s", drinks[idx].name);
    lv_obj_center(l_n);

    // 2. WIERSZ POMP (P1 + P2)
    lv_obj_t * row_pumps1 = lv_obj_create(main_col);
    lv_obj_set_size(row_pumps1, 470, 55);
    lv_obj_set_layout(row_pumps1, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row_pumps1, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_bg_opa(row_pumps1, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_pumps1, 0, 0);
    
    spinbox_p1 = create_spinbox_cell(row_pumps1, "Pompa 1", drinks[idx].pump1_percent, 0, 100, false);
    spinbox_p2 = create_spinbox_cell(row_pumps1, "Pompa 2", drinks[idx].pump2_percent, 0, 100, false);

    // 3. WIERSZ POMP (P3 + P4)
    lv_obj_t * row_pumps2 = lv_obj_create(main_col);
    lv_obj_set_size(row_pumps2, 470, 55);
    lv_obj_set_layout(row_pumps2, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row_pumps2, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_bg_opa(row_pumps2, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_pumps2, 0, 0);

    spinbox_p3 = create_spinbox_cell(row_pumps2, "Pompa 3", drinks[idx].pump3_percent, 0, 100, false);
    spinbox_p4 = create_spinbox_cell(row_pumps2, "Pompa 4", drinks[idx].pump4_percent, 0, 100, false);

    // 4. WIERSZ TOTAL
    lv_obj_t * row_total = lv_obj_create(main_col);
    lv_obj_set_size(row_total, 470, 55);
    lv_obj_set_layout(row_total, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row_total, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_total, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(row_total, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_total, 0, 0);

    spinbox_total = create_spinbox_cell(row_total, "Calkowita ilosc", drinks[idx].total_ml, 40, 400, true);
    lv_obj_set_width(lv_obj_get_parent(spinbox_total), 300); 

    // 5. PRZYCISKI
    lv_obj_t * row_btns = lv_obj_create(main_col);
    lv_obj_set_size(row_btns, 470, 50);
    lv_obj_set_layout(row_btns, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row_btns, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_btns, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(row_btns, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_btns, 0, 0);

    lv_obj_t * save_btn = lv_btn_create(row_btns); 
    lv_obj_set_size(save_btn, 140, 40); 
    lv_obj_add_style(save_btn, &style_green_btn, 0); lv_obj_add_style(save_btn, &style_focus, LV_STATE_FOCUSED);
    lv_group_add_obj(g, save_btn); lv_obj_add_event_cb(save_btn, save_btn_cb, LV_EVENT_CLICKED, (void*)idx);
    lv_obj_t * l_save = lv_label_create(save_btn); lv_label_set_text(l_save, "ZAPISZ"); lv_obj_center(l_save);

    lv_obj_t * del_btn = lv_btn_create(row_btns); 
    lv_obj_set_size(del_btn, 120, 40); 
    lv_obj_add_style(del_btn, &style_red_btn, 0); lv_obj_add_style(del_btn, &style_focus, LV_STATE_FOCUSED);
    lv_group_add_obj(g, del_btn); lv_obj_add_event_cb(del_btn, delete_btn_cb, LV_EVENT_CLICKED, (void*)idx);
    lv_obj_t * l_del = lv_label_create(del_btn); lv_label_set_text(l_del, "USUN"); lv_obj_center(l_del);
    
    lv_group_focus_obj(name_btn);
}

static void kb_event_cb(lv_event_t * e) {
    lv_obj_t * kb = lv_event_get_target(e); const char * txt = lv_keyboard_get_btn_text(kb, lv_keyboard_get_selected_btn(kb));
    if (strcmp(txt, LV_SYMBOL_OK) == 0) {
        int idx = (int)lv_event_get_user_data(e); lv_obj_t * ta = lv_keyboard_get_textarea(kb); const char * input_text = lv_textarea_get_text(ta);
        bool exists = false;
        for(int i=0; i<MAX_DRINKS; i++) { if(drinks[i].active && i != idx) { if(strcmp(drinks[i].name, input_text) == 0) { exists = true; break; } } }
        if(exists) { if(kb_label_info) lv_label_set_text(kb_label_info, "NAZWA ZAJETA!"); }
        else { strncpy(drinks[idx].name, input_text, 31); save_drinks_to_nvs(); load_edit_screen(idx); }
    }
}
void load_keyboard_screen(int drink_index) {
    clear_screen(); is_keyboard_mode = true;
    kb_label_info = lv_label_create(lv_scr_act()); lv_label_set_text(kb_label_info, "WPISZ NAZWE:"); lv_obj_align(kb_label_info, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_t * ta = lv_textarea_create(lv_scr_act()); lv_obj_set_size(ta, 400, 40); lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 30);
    lv_textarea_set_text(ta, drinks[drink_index].name); lv_group_add_obj(g, ta);
    lv_obj_t * kb = lv_keyboard_create(lv_scr_act()); lv_keyboard_set_textarea(kb, ta); lv_obj_set_size(kb, 480, 200); lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_group_add_obj(g, kb); lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_VALUE_CHANGED, (void*)drink_index); lv_group_focus_obj(kb); lv_group_set_editing(g, true);
}

static void add_new_cb(lv_event_t * e) { int id = create_new_drink(); if (id >= 0) load_keyboard_screen(id); }
static void item_click_cb(lv_event_t * e) { load_edit_screen((int)lv_event_get_user_data(e)); }
static void pour_click_cb(lv_event_t * e) { load_pouring_screen((int)lv_event_get_user_data(e)); }

void load_drink_list(bool is_edit) {
    clear_screen();
    lv_obj_t * title = lv_label_create(lv_scr_act()); lv_label_set_text(title, is_edit ? "EDYCJA" : "WYBIERZ"); lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_t * list = lv_list_create(lv_scr_act()); lv_obj_set_size(list, 440, 260); lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_add_style(list, &style_no_anim, 0);
    if (is_edit) {
        lv_obj_t * btn = lv_list_add_btn(list, LV_SYMBOL_PLUS, "DODAJ NOWY");
        lv_group_add_obj(g, btn); lv_obj_add_style(btn, &style_focus, LV_STATE_FOCUSED);
        lv_obj_add_event_cb(btn, add_new_cb, LV_EVENT_CLICKED, NULL); lv_obj_add_style(btn, &style_no_anim, 0);
    }
    for(int i=0; i<MAX_DRINKS; i++) {
        if (drinks[i].active) {
            lv_obj_t * btn = lv_list_add_btn(list, LV_SYMBOL_RIGHT, drinks[i].name);
            lv_group_add_obj(g, btn); lv_obj_add_style(btn, &style_focus, LV_STATE_FOCUSED);
            lv_obj_add_event_cb(btn, is_edit ? item_click_cb : pour_click_cb, LV_EVENT_CLICKED, (void*)i); lv_obj_add_style(btn, &style_no_anim, 0);
        }
    }
    lv_obj_t * b = lv_list_add_btn(list, LV_SYMBOL_LEFT, "WROC");
    lv_group_add_obj(g, b); lv_obj_add_style(b, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(b, (lv_event_cb_t)load_main_menu, LV_EVENT_CLICKED, NULL); lv_obj_add_style(b, &style_no_anim, 0);
    if (is_edit) { lv_group_focus_obj(lv_obj_get_child(list, 0)); }
}

static void mm_cb(lv_event_t * e) {
    const char * t = lv_list_get_btn_text(lv_obj_get_parent(lv_event_get_target(e)), lv_event_get_target(e));
    if(strcmp(t, "Nalej")==0) load_drink_list(false); 
    if(strcmp(t, "Edytuj")==0) load_drink_list(true);
    if(strcmp(t, "Plukanie")==0) load_cleaning_menu();
}
void load_main_menu() {
    clear_screen();
    
    lv_disp_t * disp = lv_disp_get_default();
    lv_theme_t * th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_ORANGE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(disp, th);

    lv_obj_t * list = lv_list_create(lv_scr_act()); lv_obj_set_size(list, 400, 260); lv_obj_center(list);
    lv_list_add_text(list, "MENU");
    lv_obj_t * b1 = lv_list_add_btn(list, LV_SYMBOL_LIST, "Nalej");
    lv_group_add_obj(g, b1); lv_obj_add_style(b1, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(b1, mm_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t * b2 = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "Edytuj");
    lv_group_add_obj(g, b2); lv_obj_add_style(b2, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(b2, mm_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t * b3 = lv_list_add_btn(list, LV_SYMBOL_REFRESH, "Plukanie"); 
    lv_group_add_obj(g, b3); lv_obj_add_style(b3, &style_focus, LV_STATE_FOCUSED);
    lv_obj_add_event_cb(b3, mm_cb, LV_EVENT_CLICKED, NULL);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_peripherals_pwm_buzzer(); 
    hx711_init();
    
    gpio_reset_pin(PIN_LCD_CS); gpio_set_direction(PIN_LCD_CS, GPIO_MODE_OUTPUT); gpio_set_level(PIN_LCD_CS, 1);
    gpio_reset_pin(PIN_LCD_DC); gpio_set_direction(PIN_LCD_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_LCD_RST); gpio_set_direction(PIN_LCD_RST, GPIO_MODE_OUTPUT);
    gpio_config_t enc_conf = { .pin_bit_mask = (1ULL<<ENC_PIN_A)|(1ULL<<ENC_PIN_B)|(1ULL<<ENC_PIN_BTN), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&enc_conf);

    spi_bus_config_t buscfg = { .miso_io_num = PIN_MISO, .mosi_io_num = PIN_MOSI, .sclk_io_num = PIN_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 320*480*3+100 };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST_ID, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_lcd = { .clock_speed_hz = 20*1000*1000, .mode = 0, .spics_io_num = PIN_LCD_CS, .queue_size = 7 };
    ESP_ERROR_CHECK(spi_bus_add_device(HOST_ID, &dev_lcd, &spi_lcd));

    ili9488_init();
    lv_init();
    load_drinks_from_nvs();

    static lv_color_t *buf1; buf1 = heap_caps_malloc(480 * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    static lv_disp_draw_buf_t disp_buf; lv_disp_draw_buf_init(&disp_buf, buf1, NULL, 480 * 10);
    static lv_disp_drv_t disp_drv; lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 480; disp_drv.ver_res = 320; 
    disp_drv.flush_cb = disp_driver_flush; disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv; lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER; indev_drv.read_cb = lvgl_encoder_read;
    lv_indev_t * enc_indev = lv_indev_drv_register(&indev_drv);
    g = lv_group_create(); lv_indev_set_group(enc_indev, g);

    // STYLE
    lv_style_init(&style_focus); 
    lv_style_set_bg_opa(&style_focus, LV_OPA_TRANSP); 
    lv_style_set_border_width(&style_focus, 3); 
    lv_style_set_border_color(&style_focus, lv_color_white());
    lv_style_set_outline_width(&style_focus, 1);
    lv_style_set_outline_color(&style_focus, lv_palette_main(LV_PALETTE_GREY));

    lv_style_init(&style_spinbox);
    lv_style_set_bg_opa(&style_spinbox, LV_OPA_TRANSP);
    lv_style_set_border_width(&style_spinbox, 0);
    lv_style_set_text_font(&style_spinbox, &lv_font_montserrat_14);
    lv_style_set_text_align(&style_spinbox, LV_TEXT_ALIGN_RIGHT);

    lv_style_init(&style_red_btn); lv_style_set_bg_color(&style_red_btn, lv_palette_main(LV_PALETTE_RED));
    lv_style_init(&style_green_btn); lv_style_set_bg_color(&style_green_btn, lv_palette_main(LV_PALETTE_GREEN));
    lv_style_init(&style_no_anim); lv_style_set_anim_time(&style_no_anim, 0); 
    
    lv_style_set_anim_time(&style_focus, 0);

    load_main_menu();
    xTaskCreate(hx711_task, "hx711_task", 2048, NULL, 5, NULL);

    const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    uint8_t old_AB = 0; int enc_accumulator = 0; int64_t last_log = 0;

    while (1) {
        old_AB <<= 2; old_AB |= ((gpio_get_level(ENC_PIN_A) << 1) | gpio_get_level(ENC_PIN_B)); old_AB &= 0x0F;
        if (enc_states[old_AB] != 0) {
            enc_accumulator += enc_states[old_AB];
            
            // DZIELNIK 4 (1 klik = 1 zmiana)
            int thr = 4; 
            
            if (enc_accumulator >= thr) { global_enc_diff++; enc_accumulator = 0; }
            else if (enc_accumulator <= -thr) { global_enc_diff--; enc_accumulator = 0; }
        }
        if (gpio_get_level(ENC_PIN_BTN) == 0) global_btn_pressed = true; else global_btn_pressed = false;

        if(esp_timer_get_time() - last_log > 500000) {
            int w_int = (int)current_weight_grams; int w_dec = (int)(fabs(current_weight_grams - w_int) * 10);
            printf("WAGA: %d.%d g\n", w_int, abs(w_dec));
            last_log = esp_timer_get_time();
        }
        lv_tick_inc(5); lv_timer_handler(); vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}