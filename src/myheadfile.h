#ifndef __MYHEADFILE_H_
#define __MYHEADFILE_H_

#include <TFT_eSPI.h>
#include "lvgl.h"
// #ifdef __cplusplus
// extern "C" {
// #endif

void mylv_init();

extern TFT_eSPI tft;
extern lv_obj_t *lvgl_led1;
extern lv_obj_t * pic1;
extern lv_obj_t *MPU_Lable;
extern int16_t encoder_moves ,encoder_pressed;

// #ifdef __cplusplus
// }
// #endif
#endif