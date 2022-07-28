#ifndef WS2812_CONTROL_H
#define WS2812_CONTROL_H
#include <stdint.h>
#include "sdkconfig.h"
#include "driver/rmt.h"

//Pulse mode names
typedef enum{
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_BLINK,
    LED_MODE_PULSE
} led_mode_t;

typedef enum{
    COLOUR_RED = 0x00FF00,
	COLOUR_GREEN = 0xFF0000,
	COLOUR_BLUE = 0x0000FF,
	COLOUR_YELLOW = 0xFFFF00,
	COLOUR_WHITE = 0xFFFFFF,
	COLOUR_AQUA = 0xFF00FF,
	COLOUR_PURPLE = 0x00FFFF
} led_colour_t; //Colours in GRB NOT RGB


//Led configuration struct
typedef struct{

    led_mode_t mode;
    uint8_t bright;
    uint16_t on_time_tics;
    uint16_t off_time_tics;
    uint16_t pulses;
    uint16_t counter;
    uint8_t state;
} LED_t;

//Main User Interface


esp_err_t LED_srv(void); //Update LED status task should be run in SRV_CLOCK intervals

//Normal LED
esp_err_t LED_init(void); 	//Initialize LED and Strip LED
esp_err_t LED_set(uint8_t led_no, uint8_t state); 		//Set normal LED on/off
esp_err_t LED_blink(uint8_t led_no, uint16_t t_on_ms, uint16_t t_off_ms, uint16_t blinks_number); // Blink LED on/off time in MS 0 beeps number means infinite

//Strip LED
esp_err_t LED_setWS(uint8_t led_no, led_colour_t colour, uint8_t brightness, uint8_t state); 	//Set strip LED ON/OFF Brightness 0-255
esp_err_t LED_blinkWS(uint8_t led_no, led_colour_t colour, uint8_t brightness, uint16_t t_on_ms, uint16_t t_off_ms, uint16_t blinks_number);	// Blink strip LED on/off time in MS 0 beeps number means infinite

//Buzzer
esp_err_t BUZZER_init(void); //Initialize Buzzer
esp_err_t BUZZER_set(uint8_t state); 	//Set buzzer LED ON/OFF
esp_err_t BUZZER_beep(uint16_t t_on_ms, uint16_t t_off_ms, uint16_t beeps_number); // Beep Buzzer on/off in MS 0 beeps number means infinite


//Backend
esp_err_t led_blink_rate(uint8_t number, uint16_t on_time_tics, uint16_t off_time_tics); //Set blink rate for LED
esp_err_t led_mode(uint8_t number, led_mode_t mode); //Set blink mode for LED
esp_err_t strip_led_colour(uint8_t number, led_colour_t colour, uint8_t brightness); //Set colour of RGB LED Brightness 0-255
esp_err_t setup_rmt_data_buffer(void); //Prepare strip leds set array for update
esp_err_t ws2812_update(void); //update strip leds
esp_err_t ws2812_control_init(void); //Strip LED init
esp_err_t strip_led_mode(uint8_t number, led_mode_t mode);
esp_err_t strip_led_blink_pulses(uint8_t number, uint16_t pulses);
esp_err_t strip_led_blink_rate(uint8_t number, uint16_t on_time_tics, uint16_t off_time_tics);
esp_err_t any_led_state(uint8_t i, uint8_t state);

#endif
