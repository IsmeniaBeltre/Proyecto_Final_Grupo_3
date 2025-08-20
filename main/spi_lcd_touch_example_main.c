/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#include "esp_lcd_ili9341.h"
/*#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#include "esp_lcd_gc9a01.h"*/
#endif

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
#include "esp_lcd_touch_stmpe610.h"
#endif

#define TAG "dispensador"

// ===== Pines =====
#define LCD_HOST            SPI2_HOST
#define PIN_NUM_SCLK        12
#define PIN_NUM_MOSI        11
#define PIN_NUM_MISO        15
#define PIN_NUM_LCD_DC      13
#define PIN_NUM_LCD_RST     14
#define PIN_NUM_LCD_CS      10
#define PIN_NUM_BK_LIGHT    22
#define PIN_NUM_TOUCH_CS    18

#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#define LCD_V_RES 320
#define LCD_H_RES 240
/*#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#define LCD_H_RES 240
#define LCD_V_RES 240*/
#endif

#define DISP_PIN1 3
#define DISP_PIN2 8
#define DISP_PIN3 9
#define DISP_PIN4 16

// ===== LVGL =====
#define LVGL_TICK_MS 2
#define LVGL_TASK_STACK 4096
#define LVGL_TASK_PRIO 2

static SemaphoreHandle_t lvgl_mux = NULL;

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
esp_lcd_touch_handle_t tp = NULL;
#endif

typedef struct { int x,y,w,h; const char* label; lv_color_t color; } Button;

#define RED     lv_color_make(255,0,0)
#define BLUE    lv_color_make(0,0,255)
#define YELLOW  lv_color_make(255,255,0)
#define CYAN    lv_color_make(0,255,255)
#define GREEN   lv_color_make(0,255,0)
#define WHITE   lv_color_make(255,255,255)
#define BLACK   lv_color_make(0,0,0)
#define DARKGREY lv_color_make(50,50,50)

Button whiskeyBtn = {40, 80, 120, 60, "Whiskey", lv_color_make(255,0,0)};
Button ronBtn     = {180, 80, 120, 60, "Ron", lv_color_make(0,0,255)};
Button cervezaBtn = {40, 160, 120, 60, "Cerveza", lv_color_make(255,255,0)};
Button vodkaBtn   = {180, 160, 120, 60, "Vodka", lv_color_make(0,255,255)};

static int currentPage = 0;

// Animación círculos
#define NUM_CIRCLES 10
typedef struct { int x,y,radius,dx,dy; lv_color_t color; } Circle;
Circle circles[NUM_CIRCLES];

// ===== Panel y Flush LVGL =====
static esp_lcd_panel_handle_t panel_handle = NULL;

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map){
    int32_t w = (area->x2 - area->x1 + 1);
    int32_t h = (area->y2 - area->y1 + 1);
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2, area->y2, color_map);
    lv_disp_flush_ready(drv);
}

// ===== Funciones de interfaz =====
void draw_welcome(lv_obj_t* scr){
    lv_obj_clean(scr);
    for(int i=0;i<NUM_CIRCLES;i++){
        lv_obj_t* circle = lv_obj_create(scr);
        lv_obj_set_size(circle, circles[i].radius*2, circles[i].radius*2);
        lv_obj_set_style_bg_color(circle, circles[i].color, LV_PART_MAIN);
        lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_pos(circle, circles[i].x, circles[i].y);
    }
    lv_obj_t* label = lv_label_create(scr);
    lv_label_set_text(label,"Bienvenido");
    lv_obj_set_style_text_font(label,&lv_font_montserrat_14,LV_PART_MAIN);
    lv_obj_center(label);
}

void animate_welcome(lv_obj_t* scr){
    for(int i=0;i<NUM_CIRCLES;i++){
        circles[i].x += circles[i].dx;
        circles[i].y += circles[i].dy;
        if(circles[i].x<0 || circles[i].x>LCD_H_RES) circles[i].dx*=-1;
        if(circles[i].y<0 || circles[i].y>LCD_V_RES) circles[i].dy*=-1;
    }
    draw_welcome(scr);
}

void draw_selection_message(lv_obj_t* scr){
    lv_obj_clean(scr);
    lv_obj_t* label = lv_label_create(scr);
    lv_label_set_text(label,"Que bebida desea");
    lv_obj_set_style_text_font(label,&lv_font_montserrat_14,LV_PART_MAIN);
    lv_obj_center(label);
}

void activate_dispenser(int pin, const char* name, int duration_ms){
    gpio_set_level(pin,1);
    lv_obj_t* scr = lv_scr_act();
    lv_obj_clean(scr);

    lv_obj_t* label = lv_label_create(scr);
    lv_label_set_text_fmt(label,"Sirviendo %s...",name);
    lv_obj_set_style_text_font(label,&lv_font_montserrat_14,LV_PART_MAIN);
    lv_obj_center(label);

    lv_obj_t* bar = lv_bar_create(scr);
    lv_obj_set_size(bar,200,20);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 50);
    lv_bar_set_range(bar,0,duration_ms);
    lv_bar_set_value(bar,0,LV_ANIM_OFF);

    unsigned long start = esp_timer_get_time()/1000;
    while( (esp_timer_get_time()/1000 - start) < duration_ms){
        unsigned long elapsed = esp_timer_get_time()/1000 - start;
        lv_bar_set_value(bar, elapsed, LV_ANIM_OFF);
        vTaskDelay(pdMS_TO_TICKS(50));
        if(xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY)==pdTRUE){
            lv_timer_handler();
            xSemaphoreGiveRecursive(lvgl_mux);
        }
    }

    gpio_set_level(pin,0);
    draw_selection_message(scr);
}

static void button_event_cb(lv_event_t * e){
    lv_obj_t* btn = lv_event_get_target(e);
    const char* label = lv_label_get_text(lv_obj_get_child(btn, 0));
    if(strcmp(label,"Whiskey")==0) activate_dispenser(DISP_PIN1,"Whiskey",15000);
    else if(strcmp(label,"Ron")==0) activate_dispenser(DISP_PIN2,"Ron",15000);
    else if(strcmp(label,"Cerveza")==0) activate_dispenser(DISP_PIN3,"Cerveza",25000);
    else if(strcmp(label,"Vodka")==0) activate_dispenser(DISP_PIN4,"Vodka",15000);
}

void draw_buttons(lv_obj_t* scr){
    Button* pageButtons[2][2] = {
        {&whiskeyBtn, &ronBtn},
        {&cervezaBtn, &vodkaBtn}
    };

    for(int i=0;i<2;i++){
        Button* b = pageButtons[currentPage][i];
        lv_obj_t* btn = lv_btn_create(scr);
        lv_obj_set_size(btn,b->w,b->h);
        lv_obj_set_pos(btn,b->x,b->y);
        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label,b->label);
        lv_obj_center(label);
        lv_obj_set_style_bg_color(btn, b->color, LV_PART_MAIN);
        lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, NULL);
    }
}

static void lvgl_task(void* arg){
    while(1){
        if(xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY)==pdTRUE){
            lv_timer_handler();
            xSemaphoreGiveRecursive(lvgl_mux);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Touch
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
static void touch_read_cb(lv_indev_drv_t * drv, lv_indev_data_t * data){
    uint16_t x[1], y[1];
    uint8_t cnt = 0;
    bool pressed = esp_lcd_touch_get_coordinates(drv->user_data, x, y, NULL, &cnt, 1);
    if(pressed && cnt>0){
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x[0];
        data->point.y = y[0];
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif


void app_main(void){
    ///gpio salidas bebiadas
    gpio_config_t gpio_cfg = { .mode = GPIO_MODE_OUTPUT };
    gpio_cfg.pin_bit_mask = (1ULL<<DISP_PIN1)|(1ULL<<DISP_PIN2)|(1ULL<<DISP_PIN3)|(1ULL<<DISP_PIN4);
    gpio_config(&gpio_cfg);
    gpio_set_level(DISP_PIN1,0); gpio_set_level(DISP_PIN2,0);
    gpio_set_level(DISP_PIN3,0); gpio_set_level(DISP_PIN4,0);

    // animacion inicio
    for(int i=0;i<NUM_CIRCLES;i++){
        circles[i].x = rand()%LCD_H_RES;
        circles[i].y = rand()%LCD_V_RES;
        circles[i].radius = 5+rand()%10;
        circles[i].dx = 1+rand()%3;
        circles[i].dy = 1+rand()%3;
        circles[i].color = lv_color_make(rand()%255,rand()%255,rand()%255);
    }

    // LVGL
    lv_init();
    static lv_disp_draw_buf_t draw_buf;
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES*20*sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES*20*sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES*20);

    // SPI 
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
    esp_lcd_new_panel_ili9341(LCD_HOST, PIN_NUM_LCD_CS, PIN_NUM_LCD_DC, PIN_NUM_LCD_RST, &panel_handle);
/*#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
    esp_lcd_new_panel_gc9a01(LCD_HOST, PIN_NUM_LCD_CS, PIN_NUM_LCD_DC, PIN_NUM_LCD_RST, &panel_handle);*/
#endif

    // LVGL driver
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.flush_cb = lvgl_flush_cb;
    lv_disp_drv_register(&disp_drv);

    // Mutex y tarea LVGL
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_task,"lvgl_task",LVGL_TASK_STACK,NULL,LVGL_TASK_PRIO,NULL);

    // Backlight
    gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BK_LIGHT, 1);

    // Pantalla
    lv_obj_t* scr = lv_scr_act();
    draw_welcome(scr);
    vTaskDelay(pdMS_TO_TICKS(2000));
    draw_selection_message(scr);
    draw_buttons(scr);

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
#endif
}
