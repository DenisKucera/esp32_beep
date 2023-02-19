
#include <stdio.h>
#include "esp_log.h"
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "soc/gpio_sig_map.h"
#include "esp_err.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include <atomic>

#define BUZZER GPIO_NUM_13

using namespace std;

void step_pulse_init(const uint32_t freq_hz, const ledc_timer_t timer_num, const gpio_num_t ledc_output, const ledc_channel_t channel)
{
    // Prepare and then apply the LEDC PWM timer configuration
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = timer_num,
        .freq_hz          = freq_hz,  // set output frequency
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = ledc_output,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_FADE_END,
        .timer_sel  = timer_num,
        .duty       = 128, // set duty at about 50%
        .hpoint     = 0,
    };    
    ledc_channel_config(&ledc_channel);
}

////////////////////////BUZZER INIT//////////////////////////
   void beep(uint beeps,uint delay, uint freq){
        step_pulse_init(freq,LEDC_TIMER_0,BUZZER,LEDC_CHANNEL_0);
        for(int i=0; i<beeps; i++){
            ledc_timer_pause(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0);
            vTaskDelay(delay / portTICK_PERIOD_MS);
            ledc_timer_resume(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0);
            vTaskDelay(delay / portTICK_PERIOD_MS);
        } 
        ledc_timer_pause(LEDC_HIGH_SPEED_MODE,LEDC_TIMER_0);
   }
extern "C" void app_main(void) {    

gpio_config_t buzzer_conf = {
        .pin_bit_mask = (1ULL<<BUZZER),   //inicializuje pin buzzer 
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
gpio_config(&buzzer_conf);

    beep(10,500,1200);

}