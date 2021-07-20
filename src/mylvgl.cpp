#include "myheadfile.h"
#include "lvgl.h"

#define DISP_HOR_RES 240
#define DISP_VER_RES 240

LV_IMG_DECLARE(Picture);
LV_IMG_DECLARE(Picture1);
LV_IMG_DECLARE(Picture2);
//输入设备的指针
lv_indev_t *indev;
//输入组指针
lv_group_t * g;

void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    // int32_t x, y;
    // for(y = area->y1; y <= area->y2; y++) {
    //     for(x = area->x1; x <= area->x2; x++) {
    //         // POINT_COLOR=color_p->full;
    //         // LCD_DrawPoint(x, y);  /* Put a pixel to the display.*/
    //         tft.fillRect(x, y, 1, 1, color_p->full);
    //         color_p++;
    //     }
    // }

    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
}

bool my_encoder_read(_lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    static int16_t encoder_pre = 0;
    data->enc_diff = encoder_moves - encoder_pre;
    encoder_pre = encoder_moves;
    // data->point.x = 120;
    // data->point.y = 30;
    if(encoder_pressed == 1) data->state = LV_INDEV_STATE_PR;
    else data->state = LV_INDEV_STATE_REL;

    return false; /*No buffering now so no more data read*/
}

void btn_event_cb(lv_obj_t * btn, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, NULL);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}
/**
 * Create a button with a label and react on Click event.
 */
void lv_ex_get_started_1(void)
{
    static lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 50, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_set_event_cb(btn, btn_event_cb);                 /*Assign a callback to the button*/

    static lv_obj_t * label = lv_label_create(btn, NULL);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/

    // static lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
    // lv_obj_set_pos(btn1, 50, 100);                            /*Set its position*/
    // lv_obj_set_size(btn1, 120, 50);                          /*Set its size*/

    // static lv_obj_t * label1 = lv_label_create(btn1, NULL);          /*Add a label to the button*/
    // lv_label_set_text(label1, "Button1");   

    lv_group_add_obj(g, btn);
    // lv_group_add_obj(g, btn1);
}

static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        printf("State: %s\n", lv_checkbox_is_checked(obj) ? "Checked" : "Unchecked");
    }
}

void lv_ex_checkbox_1(void)
{
    lv_obj_t * cb = lv_checkbox_create(lv_scr_act(), NULL);
    lv_checkbox_set_text(cb, "Checkbox Test.");
    lv_obj_align(cb, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -10);
    lv_obj_set_event_cb(cb, event_handler);
    lv_group_add_obj(g, cb);
}

lv_obj_t *lvgl_led1;

void lv_ex_led_1(void)
{
    /*Create a LED and switch it OFF*/
    lv_obj_t * led1  = lv_led_create(lv_scr_act(), NULL);
    lvgl_led1 = led1;
    lv_obj_align(led1, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);
    lv_led_off(led1);
}

void lv_ex_spinner_1(void)
{
    /*Create a Preloader object*/
    lv_obj_t * preload = lv_spinner_create(lv_scr_act(), NULL);
    lv_obj_set_size(preload, 30, 30);
    lv_obj_align(preload, NULL, LV_ALIGN_IN_TOP_RIGHT, -5, 5);
}

lv_obj_t *MPU_Label;
lv_obj_t *Time_Label;
void lv_ex_label_1(void)
{
    lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
    MPU_Label = label;
    lv_obj_t * label2 = lv_label_create(lv_scr_act(), NULL);
    Time_Label = label2;
    // lv_label_set_text(label2, "Li Ze is A Sabi");
    // lv_label_set_text_fmt(label2, "%.2f\n%.2f", 123.123, 456.789);
    // lv_obj_align(label2, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
}

lv_obj_t * pic1;
lv_obj_t * pic2;

void lv_show_picture(void)
{
    lv_obj_t * pic = lv_img_create(lv_scr_act(), NULL);
    pic1 = pic;
    lv_img_set_src(pic, &Picture2);

    lv_obj_t * pic_1 = lv_img_create(lv_scr_act(), NULL);
    pic2 = pic_1;
    lv_img_set_src(pic_1, &Picture);
    
}

void animation(void)
{
    /* INITIALIZE AN ANIMATION
    *-----------------------*/
    lv_anim_t a;
    lv_anim_init(&a);
    /* MANDATORY SETTINGS
    *------------------*/
    /*Set the "animator" function*/
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t) lv_obj_set_x); 
    /*Set the variable to animate*/
    lv_anim_set_var(&a, pic1); 
    /*Length of the animation [ms]*/
    lv_anim_set_time(&a, 5000);
    /*Set start and end values. E.g. 0, 150*/
    lv_anim_set_values(&a, 0, 240);
    /* OPTIONAL SETTINGS
    *------------------*/
    /*Time to wait before starting the animation [ms]*/
    lv_anim_set_delay(&a, 1000);
    /*Set path (curve). Default is linear*/
    // lv_anim_set_path(&a, &path);
    /*Set a callback to call when animation is ready.*/
    // lv_anim_set_ready_cb(&a, ready_cb);
    /*Set a callback to call when animation is started (after delay).*/
    // lv_anim_set_start_cb(&a, start_cb);
    /*Play the animation backward too with this duration. Default is 0 (disabled) [ms]*/
    lv_anim_set_playback_time(&a, 5000); 
    /*Delay before playback. Default is 0 (disabled) [ms]*/
    lv_anim_set_playback_delay(&a, 1000);
    /*Number of repetitions. Default is 1.  LV_ANIM_REPEAT_INFINIT for infinite repetition*/
    lv_anim_set_repeat_count(&a, 0-1);
    /*Delay before repeat. Default is 0 (disabled) [ms]*/
    lv_anim_set_repeat_delay(&a, 1000);
    /*true (default): apply the start vale immediately, false: apply start vale after delay when then anim. really starts. */
    // lv_anim_set_early_apply(&a, true);
    /* START THE ANIMATION
    *------------------*/
    lv_anim_start(&a);                             /*Start the animation*/
}

void animation1(void)
{
    /* INITIALIZE AN ANIMATION
    *-----------------------*/
    lv_anim_t a;
    lv_anim_init(&a);
    /* MANDATORY SETTINGS
    *------------------*/
    /*Set the "animator" function*/
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t) lv_obj_set_x); 
    /*Set the variable to animate*/
    lv_anim_set_var(&a, pic2); 
    /*Length of the animation [ms]*/
    lv_anim_set_time(&a, 5000);
    /*Set start and end values. E.g. 0, 150*/
    lv_anim_set_values(&a, -240, 0);
    /* OPTIONAL SETTINGS
    *------------------*/
    /*Time to wait before starting the animation [ms]*/
    lv_anim_set_delay(&a, 1000);
    /*Set path (curve). Default is linear*/
    // lv_anim_set_path(&a, &path);
    /*Set a callback to call when animation is ready.*/
    // lv_anim_set_ready_cb(&a, ready_cb);
    /*Set a callback to call when animation is started (after delay).*/
    // lv_anim_set_start_cb(&a, start_cb);
    /*Play the animation backward too with this duration. Default is 0 (disabled) [ms]*/
    lv_anim_set_playback_time(&a, 5000); 
    /*Delay before playback. Default is 0 (disabled) [ms]*/
    lv_anim_set_playback_delay(&a, 1000);
    /*Number of repetitions. Default is 1.  LV_ANIM_REPEAT_INFINIT for infinite repetition*/
    lv_anim_set_repeat_count(&a, 0-1);
    /*Delay before repeat. Default is 0 (disabled) [ms]*/
    lv_anim_set_repeat_delay(&a, 1000);
    /*true (default): apply the start vale immediately, false: apply start vale after delay when then anim. really starts. */
    // lv_anim_set_early_apply(&a, true);
    /* START THE ANIMATION
    *------------------*/
    lv_anim_start(&a);                             /*Start the animation*/
}

static lv_obj_t * kb;
static lv_obj_t * ta;

static void kb_event_cb(lv_obj_t * keyboard, lv_event_t e)
{
    lv_keyboard_def_event_cb(kb, e);
    if(e == LV_EVENT_CANCEL) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_del(kb);
        kb = NULL;
    }
}

static void kb_create(void)
{
    kb = lv_keyboard_create(lv_scr_act(), NULL);
    lv_keyboard_set_cursor_manage(kb, true);
    lv_obj_set_event_cb(kb, kb_event_cb);
    lv_keyboard_set_textarea(kb, ta);

}

static void ta_event_cb(lv_obj_t * ta_local, lv_event_t e)
{
    if(e == LV_EVENT_CLICKED && kb == NULL) {
        kb_create();
    }
}

void lv_ex_keyboard_1(void)
{
    /*Create a text area. The keyboard will write here*/
    ta  = lv_textarea_create(lv_scr_act(), NULL);
    lv_obj_align(ta, NULL, LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 16);
    lv_obj_set_event_cb(ta, ta_event_cb);
    lv_textarea_set_text(ta, "");
    lv_coord_t max_h = LV_VER_RES / 2 - LV_DPI / 8;
    if(lv_obj_get_height(ta) > max_h) lv_obj_set_height(ta, max_h);
    kb_create();
    lv_group_add_obj(g, ta);
    lv_group_add_obj(g, kb);
}

void mylv_init(void)
{
    lv_init();
    static lv_disp_buf_t disp_buf;
    static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX / 10];                     /*Declare a buffer for 1/10 screen size*/
    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX / 10);    /*Initialize the display buffer*/

    static lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    static lv_indev_drv_t indev_drv;                  /*Descriptor of a input device driver*/
    lv_indev_drv_init(&indev_drv);             /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_ENCODER;    /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = my_encoder_read;      /*Set your driver function*/
    indev = lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/
    g = lv_group_create();
    lv_indev_set_group(indev, g);

    lv_ex_get_started_1();
    // // lv_ex_led_1();
    lv_ex_spinner_1();
    lv_ex_label_1();
    lv_ex_checkbox_1();
    // lv_show_picture();
    // animation();
    // lv_ex_spinner_1();
    // BuildPages();
    // setup_ui(&guider_ui);
    // lv_ex_keyboard_1();

}
