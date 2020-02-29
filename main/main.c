#include <stdio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_camera.h"

#include "display.h"

extern const uint8_t eye_raw_image_data[] asm("_binary_eye_raw_start");

typedef struct {
	int px; //x pos of pupil
	int py; //y pos of pupil
	int pc; //status: 0 open 1 half-closed, 2 fully-closed blink
} imgdesc_t;

const imgdesc_t images[]={
	{257, 197, 0},
	{257, 197, 1},
	{257, 197, 2},
	{250, 207, 0},
	{240, 205, 0},
	{232, 208, 0},
	{231, 208, 0},
	{231, 208, 1},
	{231, 208, 2},
	{218, 217, 0},
	{214, 225, 0},
	{214, 225, 1},
	{214, 225, 2},
	{206, 250, 0},
	{206, 250, 1},
	{206, 250, 2},
	{208, 259, 0},
	{212, 270, 0},
	{212, 270, 1},
	{212, 270, 2},
	{220, 276, 0},
	{224, 279, 0},
	{228, 281, 0},
	{228, 281, 1},
	{228, 281, 2},
	{254, 294, 0},
	{254, 294, 1},
	{254, 294, 2},
	{292, 285, 0},
	{292, 285, 1},
	{292, 285, 2},
	{318, 268, 0},
	{318, 268, 1},
	{318, 268, 2},
	{326, 248, 0},
	{326, 248, 1},
	{324, 225, 0},
	{331, 219, 0},
	{336, 219, 0},
	{336, 219, 1},
	{336, 219, 2},
	{314, 193, 0},
	{314, 193, 1},
	{314, 193, 2},
	{281, 193, 0},
	{273, 192, 0},
	{273, 192, 1},
	{273, 192, 2},
	{241, 200, 0},
	{241, 200, 1},
	{241, 200, 2},
	{253, 211, 0},
	{250, 210, 0},
	{250, 210, 1},
	{250, 210, 2},
	{224, 217, 0},
	{224, 226, 0},
	{224, 226, 1},
	{224, 226, 2},
	{223, 226, 0},
	{218, 243, 0},
	{218, 243, 1},
	{218, 243, 2},
	{216, 257, 0},
	{223, 265, 0},
	{227, 262, 0},
	{227, 262, 1},
	{227, 262, 2},
	{234, 284, 0},
	{234, 284, 1},
	{234, 284, 2},
	{269, 283, 0},
	{269, 283, 1},
	{269, 283, 2},
	{288, 285, 0},
	{295, 284, 0},
	{295, 284, 1},
	{295, 284, 2},
	{321, 264, 0},
	{309, 268, 0},
	{324, 242, 0},
	{324, 242, 1},
	{324, 242, 2},
	{322, 213, 0},
	{308, 220, 0},
	{308, 220, 1},
	{308, 220, 2},
	{261, 216, 0},
	{250, 215, 0},
	{250, 215, 1},
	{250, 215, 2},
	{237, 224, 0},
	{237, 224, 1},
	{237, 224, 2},
	{227, 245, 0},
	{236, 234, 0},
	{236, 234, 1},
	{236, 234, 2},
	{248, 241, 0},
	{248, 241, 1},
	{248, 241, 2},
	{269, 238, 0},
	{285, 235, 0},
	{285, 235, 1},
	{285, 235, 2},
	{310, 238, 0},
	{310, 238, 1},
	{310, 238, 2},
	{341, 240, 0},
	{341, 240, 1},
	{341, 240, 2},
	{328, 259, 0},
	{328, 259, 1},
	{328, 259, 2},
	{295, 256, 0},
	{306, 252, 0},
	{306, 252, 1},
	{306, 252, 2},
	{260, 254, 0},
	{260, 254, 1},
	{260, 254, 2},
	{239, 255, 0},
	{239, 255, 1},
	{239, 255, 2},
	{227, 258, 0},
	{227, 258, 1},
	{227, 258, 2},
	{243, 277, 0},
	{253, 276, 0},
	{253, 276, 1},
	{253, 276, 2},
	{279, 276, 0},
	{279, 276, 1},
	{279, 276, 2},
	{288, 262, 0},
	{288, 262, 1},
	{288, 262, 2},
	{313, 266, 0},
	{313, 266, 1},
	{313, 266, 2},
	{278, 285, 0},
	{263, 281, 0},
	{263, 281, 1},
	{263, 281, 2},
	{242, 278, 0},
	{242, 278, 1},
	{242, 278, 2},
	{319, 236, 0},
	{340, 236, 0},
	{0,0,-1} //final
};

#define EYE_XMIN 190
#define EYE_XMAX 370
#define EYE_YMIN 180
#define EYE_YMAX 310

//QQVGA
#define CAMW 160
#define CAMH 120

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



int eye_find(int x, int y, int p) {
	//quick transform qqvga coords -> eye pupil coords.
	x=(x*(EYE_XMAX-EYE_XMIN))/CAMH+EYE_XMIN;
	y=(y*(EYE_YMAX-EYE_YMIN))/CAMW+EYE_YMIN;
	printf("Eye %d %d\n", x, y);
	//find image with given status with pupil closest to x,y
	int mindist=999999999;
	int minno=0;
	for (int t=0; images[t].pc>=0; t++) {
		if (images[t].pc==p) {
			int dx=images[t].px-x;
			int dy=images[t].py-y;
			int d=dx*dx+dy*dy;
			if (d<mindist) {
				mindist=d;
				minno=t;
			}
		}
	}
	printf("Eye % 3d % 3d dist % 3d % 3d\n", x, y, x-images[minno].px, y-images[minno].py);
	return minno;
}

void calc_mask(uint8_t *mask) {
	camera_fb_t *pic;
	int iter=32;
	int intens=192;
	int16_t *w=calloc(CAMW, CAMH*2);

	for (int y=0; y<16; y++) {
		for (int x=0; x<16; x++) {
			int c=0;
			if (x>=6 && x<10 && y>=4 && y<8) c=intens;
			display_setpixel(15-y, x, c);
		}
	}
	display_flip();
	vTaskDelay(200 / portTICK_RATE_MS);
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	for (int i=0; i<iter; i++) {
		pic = esp_camera_fb_get(); //note: gets 160x120 picture
		for (int y=0; y<CAMW*CAMH; y++) {
			w[y]+=pic->buf[y];
		}
		esp_camera_fb_return(pic);
	}


	for (int y=0; y<16; y++) {
		for (int x=0; x<16; x++) {
			int c=0;
			if (x==3 || x==12 || y==3 || y==12) c=intens;
			display_setpixel(15-y, x, c);
		}
	}
	display_flip();
	vTaskDelay(200 / portTICK_RATE_MS);
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	for (int i=0; i<iter; i++) {
		pic = esp_camera_fb_get(); //note: gets 160x120 picture
		for (int y=0; y<CAMW*CAMH; y++) {
			if (w[y]<253*iter) w[y]-=pic->buf[y];
		}
		esp_camera_fb_return(pic);
	}

	int avg=0;
	for (int y=0; y<CAMW*CAMH; y++) {
		if (w[y]<0) w[y]=0;
		avg+=w[y];
	}
	avg=avg/(CAMW*CAMH);


	for (int y=0; y<CAMW*CAMH; y++) {
		if (w[y]<avg*0.3) {
			mask[y]=0;
		} else if (w[y]>253*iter) {
			mask[y]=255;
		} else {
			mask[y]=w[y]/iter;
		}
	}
}


void app_main(void) {

	display_init();
	printf("Display inited.\n");
	esp_camera_init(&camera_config);
	printf("Camera inited.\n");
	sensor_t *sensor=esp_camera_sensor_get();
	sensor->set_gain_ctrl(sensor, 0);
	sensor->set_exposure_ctrl(sensor, 0);
	sensor->set_agc_gain(sensor, 1); //0-30
	sensor->set_aec_value(sensor, 100); //0-1200
	
	uint8_t *prev_img, *diff_img, *mask;
	prev_img=calloc(CAMW, CAMH);
	diff_img=calloc(CAMW, CAMH);
	mask=calloc(CAMW, CAMH);
	
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_INPUT,
		.pull_up_en=1,
		.pin_bit_mask=(1<<2)|(1<<4)
	};
	gpio_config(&io_conf);
	
	int mode=2;
	
	camera_fb_t *pic;
	
	calc_mask(mask);
	int blink_timer=10;

	float eye_x=100, eye_y=100;
	while(1) {
		if (gpio_get_level(GPIO_NUM_2)==0) {
			mode++;
			printf("Mode %d\n", mode);
			if (mode==4) mode=0;
			while(gpio_get_level(GPIO_NUM_2)==0) {
				vTaskDelay(100 / portTICK_RATE_MS);
			}
		}


		pic = esp_camera_fb_get(); //note: gets 160x120 picture
		if (pic==NULL) {
			printf("Huh, no frame?\n");
			continue;
		}
		printf("Got frame\n");

		//Remove mask
		for (int y=0; y<CAMH; y++) {
			for (int x=0; x<CAMW; x++) {
				int d=pic->buf[y*CAMW+x]-mask[y*CAMW+x];
				if (d<0) {
					d=0;
				}
				pic->buf[y*CAMW+x]=d;
			}
		}

		//Calculate difference
		int cx, cy, cn;
		cx=0; cy=0; cn=0;
		for (int y=0; y<CAMH; y++) {
			for (int x=0; x<CAMW; x++) {
				int d=prev_img[y*CAMW+x]-pic->buf[y*CAMW+x];
				d=(d*d)/32; //square of the difference
				if (d>255) d=255; //clip
				diff_img[y*CAMW+x]=d;
				//note again: x nd y are exchanged because the camera is rotated
				cx+=y*d;
				cy+=x*d;
				cn+=d;
			}
		}
		if (cn) { //don't divide by zero
			cx=cx/cn;
			cy=cy/cn;
		}
		memcpy(prev_img, pic->buf, CAMW*CAMH);

		//Use cx/cy/cn to find new eye pos.
		//Note we can use cn as a movement amount indicator. 
		
		if (cn>5000) {
			float th=(cn/200000.0);
			if (th>1) th=1;
			eye_x=eye_x*(1.0-th)+cx*th;
			eye_y=eye_y*(1.0-th)+cy*th;
		}
		if (blink_timer==0) blink_timer=50+(rand()%30);
		blink_timer--;

		int eye_idx=eye_find(eye_x, eye_y, blink_timer==1?2:blink_timer<3?1:0);
		printf("cx %d cy %d cn %d ei %d\n", cx, cy, cn, eye_idx);

		const uint8_t *drawfb=diff_img;
		if (mode==0) drawfb=pic->buf;
		if (mode==1) drawfb=diff_img;
		if (mode==2) drawfb=&eye_raw_image_data[eye_idx*256];
		if (mode==3) drawfb=mask;
		for (int x=0; x<16; x++) {
			for (int y=0; y<16; y++) {
				if (mode==2) {
					display_setpixel(15-y, x, *drawfb++);
				} else {
					//Scale image
					int sc=0;
					for (int sy=0; sy<7; sy++) {
						for (int sx=0; sx<10; sx++) {
							sc+=drawfb[(y*7+sy)*160+(x*10+sx)];
						}
					}
					//note: camera is 45 degrees rotated wrt display
					display_setpixel(15-y, x, sc/70);
				}
			}
		}

	
		if (mode!=2) display_setpixel(15-cx/7, cy/10, 255);

		display_flip();
		esp_camera_fb_return(pic);
	}
}

