
/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */


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

//Make 1 to output picture data on the serial port, in base64.
//Picture is encoded as 160x120 8-bit grayscale raw image data.
#define DUMP_PIC_SERIAL 0

//This refers the raw set of 16x16 images.
extern const uint8_t eye_raw_image_data[] asm("_binary_eye_raw_start");

typedef struct {
	int px; //x pos of pupil
	int py; //y pos of pupil
	int pc; //status: 0 open 1 half-closed, 2 fully-closed blink
} imgdesc_t;

//This array contains the information for each 16x16 image in eye_raw_image_data: we use
//this info to pick the correct image to display for a given center of movement and blink status.
//Note pupil x/y is in the *original, unscaled* image data, and as such is more-or-less used as
//an indication.
//Note that this has been mostly autogenerated, see eye_gfx/readme.txt for more info
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

//Parameters to tweak the mapping between the pupil xpos and ypos and the corresponding
//center of movement.
#define EYE_XMIN 200
#define EYE_XMAX 390
#define EYE_YMIN 170
#define EYE_YMAX 320

//QQVGA
#define CAMW 160
#define CAMH 120

//Debug function to base64-encode an image or other data
static void __attribute__((unused)) base64_dump(const unsigned char *src, size_t len, int *ll) {
	static const unsigned char base64_table[65] =
			"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	const unsigned char *end, *in;
	int line_len;

	end = src + len;
	in = src;
	if (ll) line_len = *ll; else line_len=0;
	while (end - in >= 3) {
		putchar(base64_table[in[0] >> 2]);
		putchar(base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)]);
		putchar(base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)]);
		putchar(base64_table[in[2] & 0x3f]);
		in += 3;
		line_len += 4;
		if (line_len >= 72) {
			putchar('\n');
			line_len = 0;
		}
	}

	if (end - in) {
		putchar(base64_table[in[0] >> 2]);
		if (end - in == 1) {
			putchar(base64_table[(in[0] & 0x03) << 4]);
			putchar('=');
		} else {
			putchar(base64_table[((in[0] & 0x03) << 4) |
					      (in[1] >> 4)]);
			putchar(base64_table[(in[1] & 0x0f) << 2]);
		}
		putchar('=');
		line_len += 4;
	}
	if (ll) *ll=line_len;
}

//Camera config is specific for an esp-cam board
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


//For a given center of movement (indicated by x and y) and a blink status (indicated by p), this
//returns the index of the most applicable raw image.
int eye_find(int x, int y, int p) {
	//quick transform qqvga coords -> eye pupil coords.
	x=(x*(EYE_XMAX-EYE_XMIN))/CAMH+EYE_XMIN;
	y=(y*(EYE_YMAX-EYE_YMIN))/CAMW+EYE_YMIN;
	//printf("Eye %d %d\n", x, y);
	//find image with given status with pupil closest to x,y
	int mindist=999999999;
	int minno=0;
	//Loop over all images, find best.
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
	//printf("Eye % 3d % 3d dist % 3d % 3d\n", x, y, x-images[minno].px, y-images[minno].py);
	return minno;
}

//Helper function: add an int to an 16-bit value, but make sure to never overflow.
inline void add_clip16(int16_t *v, int w) {
	if (w>0 && *v<32767-w) *v+=w;
	if (w<0 && *v>-32767-w) *v+=w;
}

/*
This function calculates a mask to filter out the fact that the camera can see some LEDs. It takes
picures while the LEDs around the camera are on and pictures while LEDs away from the camera are on,
and substracts the two in order to generate a mask that allows us to filter out these pixels. In the
resulting mask, 0 means the pixel is normal, 255 means it's entirely obscured by an LED.
*/
void calc_mask(uint8_t *mask) {
	camera_fb_t *pic;
	int iter=32;
	int intens=192;
	//Allocate work buffers
	int16_t *w=calloc(CAMW, CAMH*2);
	int16_t *w2=calloc(CAMW, CAMH*2);

	//Turn on LEDs around camera
	for (int y=0; y<16; y++) {
		for (int x=0; x<16; x++) {
			int c=0;
			if (x>=6 && x<10 && y>=4 && y<8) c=intens;
			display_setpixel(15-y, x, c);
		}
	}
	display_flip();
	vTaskDelay(200 / portTICK_RATE_MS);
	//throw away some frames as they're potentially made before the LEDs turned on
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	printf("Mask: sample with leds on\n");
	//Take and average pictures
	for (int i=0; i<iter; i++) {
		pic = esp_camera_fb_get(); //note: gets 160x120 picture
#if DUMP_PIC_SERIAL
		if (i==iter-1) {
			base64_dump(pic->buf, CAMW*CAMH, NULL);
			printf("\n\n");
		}
#endif
		for (int y=0; y<CAMW*CAMH; y++) {
			w[y]+=pic->buf[y];
		}
		esp_camera_fb_return(pic);
	}
	printf("Sampled.\n");

	//Turn on two lines of LEDs away from the camera
	for (int y=0; y<16; y++) {
		for (int x=0; x<16; x++) {
			int c=0;
			if (x==3 || x==12) c=intens;
			display_setpixel(15-y, x, c);
		}
	}
	display_flip();
	vTaskDelay(200 / portTICK_RATE_MS);
	//throw away some frames as they're potentially made before the LEDs turned on
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	esp_camera_fb_return(esp_camera_fb_get());
	printf("Mask: sample with leds off\n");
	//Sample a few pictures
	for (int i=0; i<iter; i++) {
		pic = esp_camera_fb_get(); //note: gets 160x120 picture
#if DUMP_PIC_SERIAL
		if (i==iter-1) {
			base64_dump(pic->buf, CAMW*CAMH, NULL);
			printf("\n\n");
		}
#endif
		for (int y=0; y<CAMW*CAMH; y++) {
			w[y]-=pic->buf[y];
		}
		esp_camera_fb_return(pic);
	}


	int avg=0;

	//We
	for (int smooth_iter=0; smooth_iter<12; smooth_iter++) {
		//First of all, calculate the average pixel value.
		avg=0;
		for (int y=0; y<CAMW*CAMH; y++) {
			if (w[y]<0) w[y]=0; //disregard negative pixel values
			avg+=w[y];
		}
		avg=avg/(CAMW*CAMH);
		memcpy(w2, w, CAMW*CAMH*2); //save old image so we can reference back to it while writing new one

		//This does some smoothing... effectively, if a pixel is very affected by the LEDs, it also
		//spreads this over the surrounding pixels. The idea is to make sure random camera noise does
		//not influence the mask too much.
		for (int y=1; y<CAMH-1; y++) {
			for (int x=1; x<CAMW-1; x++) {
				int d=0;
				d=w2[y*CAMW+x]-avg;
				//if (w2[y*CAMW+x]>avg*2) d=64*iter;
				//if (w2[y*CAMW+x]<avg*0.1) d=-64*iter;

				add_clip16(&w[(y-1)*CAMW+(x+1)], d);
				add_clip16(&w[(y-1)*CAMW+x], d);
				add_clip16(&w[(y-1)*CAMW+(x-1)], d);
				add_clip16(&w[(y)*CAMW+(x+1)], d);
				add_clip16(&w[(y)*CAMW+(x)], d);
				add_clip16(&w[(y)*CAMW+(x-1)], d);
				add_clip16(&w[(y+1)*CAMW+(x+1)], d);
				add_clip16(&w[(y+1)*CAMW+x], d);
				add_clip16(&w[(y+1)*CAMW+(x-1)], d);
			}
		}
	}

	//Re-calculate average
	avg=0;
	for (int y=0; y<CAMW*CAMH; y++) {
		if (w[y]<0) w[y]=0;
		avg+=w[y];
	}
	avg=avg/(CAMW*CAMH);

	//Increase the contrast of the mask: if very low difference, make 0, if very high difference make 255,
	//otherwise keep the (normalized to 0-255) value.
	for (int y=0; y<CAMW*CAMH; y++) {
		if (w[y]<avg*0.3) {
			mask[y]=0;
		} else if (w[y]>253*iter) {
			mask[y]=255;
		} else {
			mask[y]=w[y]/iter;
		}
	}
#if DUMP_PIC_SERIAL
	base64_dump(mask, CAMW*CAMH, NULL);
	printf("\n\n");
#endif
	free(w);
	free(w2);
}

int cam_gain, cam_exposure;

//Very simple automatic gain and exposure adjustment.
//Takes a frame, counts the 'hot' and 'dark' pixels and tweaks exposure and gain to make
//sure both are more-or-less in balance. incr indicates how fast we should modify the
//values - higher means larger steps to compensate.
//Note that the added value here is that it can be passed a mask that masks out all the
//pixels that are looking directly at a LED - by doing that, the exposure won't fly
//up and down because those LEDs happen to be on or off.
void adjust_exposure(sensor_t *sensor, uint8_t *mask, uint8_t *fb, int incr) {
	int under=0;
	int over=0;
	//Count under- and over-exposed pixels
	for (int y=0; y<CAMW*CAMH; y++) {
		if (mask[y]<128) {
			if (fb[y]<4) under++;
			if (fb[y]>251) over++;
		}
	}
	//Adjust exposure based on that.
	if (under>20) cam_exposure+=incr;
	if (over>20) cam_exposure-=incr;
	//If exposure is too high or low, tweak gain instead.
	if (cam_exposure>1000) {
		cam_exposure=900;
		cam_gain++;
	}
	if (cam_exposure<10) {
		cam_exposure=100;
		cam_gain--;
	}
	//printf("Under %d over %d Exp %d gain %d\n", under, over, cam_exposure, cam_gain);
	sensor->set_agc_gain(sensor, cam_gain); //0-30
	sensor->set_aec_value(sensor, cam_exposure); //0-1200
}

//This is used to quickly calculate the initial exposure after the camera is turned
//on. It takes a bunch of pics and uses them to quickly (depending on chg) tune in
//to what should be more-or-less the correct values.
void calc_initial_exposure(sensor_t *sensor, uint8_t *mask, int iter, int chg) {
	camera_fb_t *pic;
	//Turn off all LEDs
	for (int y=0; y<16; y++) {
		for (int x=0; x<16; x++) {
			display_setpixel(15-y, x, 0);
		}
	}
	display_flip();

	for (int i=0; i<iter; i++) {
		pic = esp_camera_fb_get(); //note: gets 160x120 picture
		adjust_exposure(sensor, mask, pic->buf, chg);
		esp_camera_fb_return(pic);
	}
}


//The heart of the code. It's kinda spagetti-ey, sorry for that.
void app_main(void) {
	esp_camera_init(&camera_config);
	printf("Camera inited.\n");
	display_init();
	printf("Display inited.\n");
	sensor_t *sensor=esp_camera_sensor_get();
	sensor->set_gain_ctrl(sensor, 0);
	sensor->set_exposure_ctrl(sensor, 0);
	cam_gain=1;
	cam_exposure=500;
	
	//Alocate images for motion detecting
	uint8_t *prev_img, *diff_img, *mask;
	prev_img=calloc(CAMW, CAMH);
	diff_img=calloc(CAMW, CAMH);
	mask=calloc(CAMW, CAMH);
	
	//buttons
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_INPUT,
		.pull_up_en=1,
		.pin_bit_mask=(1<<2)|(1<<4)
	};
	gpio_config(&io_conf);
	
	//Display mode: you can press a button to go into a different mode to
	//see what the motion sensing algo does
	int mode=2;
	
	camera_fb_t *pic;
	
	printf("Calculating mask\n");
	calc_mask(mask);

	//large steps
	calc_initial_exposure(sensor, mask, 10, 30);
	//fine tune
	calc_initial_exposure(sensor, mask, 10, 5);

	//Holds the time until the next blink
	int blink_timer=10;
	//Amount of frames the motion detection code should ignore after a blink or an eye movement
	int blink_ignore_cam=11;

	//starting positions of eye
	float eye_x=100, eye_y=100;
	int eye_idx=0, eye_idx_old=0;
	while(1) {
		//Handle button presses
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
//		printf("Got frame\n");

		//Remove masked-out pixels
		for (int y=0; y<CAMH; y++) {
			for (int x=0; x<CAMW; x++) {
				int d=(int)pic->buf[y*CAMW+x] - (int)mask[y*CAMW+x];
				if (d<0) {
					d=0;
				}
				pic->buf[y*CAMW+x]=d;
			}
		}

		//Calculate difference between this and prev frame
		int avgdif;
		avgdif=0;
		for (int y=0; y<CAMH; y++) {
			for (int x=0; x<CAMW; x++) {
				int d=prev_img[y*CAMW+x]-pic->buf[y*CAMW+x];
				d=(d*d)/32; //square of the difference
				if (d>255) d=255; //clip
				diff_img[y*CAMW+x]=d;
				avgdif+=d;
			}
		}
		//Calculate average squared difference. This is an indicator of how much actually moved
		//in the scene.
		avgdif=avgdif/(CAMH*CAMW);
		//printf("avgdif %d\n", avgdif);

		//figure out center of movement
		//We do that by taking the average of all coordinates weighted by the difference. We ignore
		//pixels that changed less than the average, to keep out noise.
		int cx, cy, cn;
		cx=0; cy=0; cn=0;
		for (int y=0; y<CAMH; y++) {
			for (int x=0; x<CAMW; x++) {
				int d=diff_img[y*CAMW+x];
				if (d>avgdif+2) {
					//note again: x and y are exchanged because the camera is rotated
					cx+=y*d;
					cy+=x*d;
					cn+=d;
				}
			}
		}

		if (cn) { //don't divide by zero
			cx=cx/cn;
			cy=cy/cn;
		}
		//Save current image for next loop.
		memcpy(prev_img, pic->buf, CAMW*CAMH);

		//Use cx/cy/cn to find new eye pos.
		//Note we can use cn as a movement amount indicator. 
		if (cn>5000 && blink_ignore_cam==0) {
			float th=(cn/200000.0);
			if (th>1) th=1;
			eye_x=eye_x*(1.0-th)+cx*th;
			eye_y=eye_y*(1.0-th)+cy*th;
		}
		if (blink_ignore_cam!=0) blink_ignore_cam--;
		if (blink_timer==3) blink_ignore_cam=5;
		if (blink_timer==0) blink_timer=50+(rand()%30);
		blink_timer--;

		eye_idx_old=eye_idx;
		eye_idx=eye_find(eye_x, eye_y, blink_timer==1?2:blink_timer<3?1:0);
		//Ignore a frame or two if the eye image changed, so we don't trigger off reflections.
		if (eye_idx_old!=eye_idx) blink_ignore_cam=2;
//		printf("cx %d cy %d cn %d ei %d\n", cx, cy, cn, eye_idx);

		//Depending on mode, we draw something different.
		const uint8_t *drawfb=diff_img;
		if (mode==0) drawfb=pic->buf;	//Mode 0: camera view
		if (mode==1) drawfb=diff_img;	//Mode 1: motion detection view
		if (mode==2) drawfb=&eye_raw_image_data[eye_idx*256];	//Mode 2: main eye view
		if (mode==3) drawfb=mask;	//Mode 3: inspect mask
		//Draw selected image on LEDs
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

		//On the debug images, show the center of motion as well as the current eye position.
		if (mode!=2) display_setpixel(15-cx/7, cy/10, 128);
		if (mode!=2) display_setpixel(15-eye_x/7, eye_y/10, 64);
		//Show frame
		display_flip();
		adjust_exposure(sensor, mask, pic->buf, 1);
		esp_camera_fb_return(pic);
	}
}

