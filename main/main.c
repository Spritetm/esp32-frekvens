#include <stdio.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_camera.h"

#include "display.h"

static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = CONFIG_RESET,
    .pin_xclk = CONFIG_XCLK,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,

    .pin_d7 = CONFIG_D7,
    .pin_d6 = CONFIG_D6,
    .pin_d5 = CONFIG_D5,
    .pin_d4 = CONFIG_D4,
    .pin_d3 = CONFIG_D3,
    .pin_d2 = CONFIG_D2,
    .pin_d1 = CONFIG_D1,
    .pin_d0 = CONFIG_D0,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_pclk = CONFIG_PCLK,

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = CONFIG_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QQVGA,       //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 2       //if more than one, i2s runs in continuous mode. Use only with JPEG
};


void app_main(void) {
	display_init();
	printf("Display inited.\n");
	esp_camera_init(&camera_config);
	printf("Camera inited.\n");
	while(1) {
		camera_fb_t *pic = esp_camera_fb_get(); //note: gets 160x120 picture
		if (pic==NULL) {
			printf("Huh, no frame?\n");
			continue;
		}
		printf("Got frame\n");

		for (int x=0; x<16; x++) {
			for (int y=0; y<16; y++) {
				//Scale image
				int sc=0;
				for (int sy=0; sy<7; sy++) {
					for (int sx=0; sx<10; sx++) {
						sc+=pic->buf[(y*7+sy)*160+(x*10+sx)];
					}
				}
				//note: camera is 45 degrees rotated wrt display

				display_setpixel(15-y, x, sc/70);
			}
		}
		display_flip();
		esp_camera_fb_return(pic);
	}
}

