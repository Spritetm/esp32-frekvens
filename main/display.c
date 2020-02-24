
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_types.h"
#include "val2pwm.h"

//Frekvens uses 16 SCT2024 LED drivers
//http://www.starchips.com.tw/pdf/datasheet/SCT2024V01_03.pdf

#define GPIO_LAK 14
#define GPIO_CLK 15
#define GPIO_DA 13
#define GPIO_EN 12


#define BITPLANE_CT 8

#define BAM_DIV 50 //1MHz/(BAM_DIV*(1<<bit))

static volatile int bam_bit;
static SemaphoreHandle_t bam_sema = NULL;
static uint16_t bam_bitplane_a[BITPLANE_CT*16];
static uint16_t bam_bitplane_b[BITPLANE_CT*16];
static uint16_t *bam_bitplane_cur=bam_bitplane_a;
static uint16_t *bam_bitplane_next=bam_bitplane_a;
static uint16_t *bam_bitplane_draw=bam_bitplane_a;


static void IRAM_ATTR bam_timer_isr(void *para) {
	//Latch in what the shifter task shifted out.
	gpio_set_level(GPIO_LAK, 1);
	ets_delay_us(1);
	gpio_set_level(GPIO_LAK, 0);
	ets_delay_us(1);

	timer_spinlock_take(TIMER_GROUP_0);

	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);

	//Set alarm value according what current BAM bit is
	timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, (1<<bam_bit)*BAM_DIV);

	/* After the alarm has been triggered
	  we need enable it again, so it is triggered the next time */
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

	timer_spinlock_give(TIMER_GROUP_0);


	//update bam_bit for what is going to be sent next
	bam_bit=bam_bit+1;
	if (bam_bit==BITPLANE_CT) {
		bam_bit=0;
	}

	/* Now just send the event data back to the main program task */
	xSemaphoreGiveFromISR(bam_sema, NULL);

	//All done.
	portYIELD_FROM_ISR();
}



static void display_task(void *arg) {
	spi_device_handle_t spi;
	esp_err_t ret;

	//latch is used as a gpio in the timer int
	gpio_pad_select_gpio(GPIO_LAK);
	gpio_set_direction(GPIO_LAK, GPIO_MODE_OUTPUT);
	//always enable outputs
	gpio_pad_select_gpio(GPIO_EN);
	gpio_set_direction(GPIO_EN, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_EN, 0); //high active

	bam_bit=0;

	spi_bus_config_t buscfg={
		.miso_io_num=-1,
		.mosi_io_num=GPIO_DA,
		.sclk_io_num=GPIO_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=16*16
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=10*1000*1000,			//Clock out at 10 MHz
		.mode=0,								//SPI mode 0
		.spics_io_num=-1,						//we'll handle latch in the timer
		.queue_size=2,
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 2);
	ESP_ERROR_CHECK(ret);
	//Attach the device to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	
	bam_sema=xSemaphoreCreateBinary();
	
	/* Select and initialize basic parameters of the timer */
	timer_config_t config={
		.divider = 80, //1MHz
		.counter_dir = TIMER_COUNT_UP,
		.counter_en = TIMER_PAUSE,
		.alarm_en = TIMER_ALARM_EN,
		.intr_type = TIMER_INTR_LEVEL,
		.auto_reload = 1, //auto-reset on int
//		.clk_src = TIMER_SRC_CLK_APB,
	};
	timer_init(TIMER_GROUP_0, TIMER_0, &config);
	//counter start
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000);
	timer_isr_register(TIMER_GROUP_0, TIMER_0, bam_timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
	timer_start(TIMER_GROUP_0, TIMER_0);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);

	spi_transaction_t trans={
		.length=16*16
	};

	while(1) {
		//Send the next bitplane
		trans.tx_buffer=&bam_bitplane_cur[bam_bit*16];
		ret=spi_device_polling_transmit(spi, &trans);  //Transmit!
		assert(ret==ESP_OK);            //Should have had no issues.
		if (bam_bit==0) {
			//buffer flip, if needed
			bam_bitplane_cur=bam_bitplane_next;
		}
		//wait for timer to latch this data
		xSemaphoreTake(bam_sema, portMAX_DELAY);
	}
}

void display_setpixel(int x, int y, int v) {
	//Refrobnicating because pixels are located weirdly
	int bx=(x&7);
	int by=y/2;
	if (y&1) bx+=8;
	if ((x&8)==0) by+=8;
	v=valToPwm(v)>>(16-BITPLANE_CT);

	int bitmask=1;
	int pixmask=(1<<bx);
	for (int i=0; i<BITPLANE_CT; i++) {
		if (v&bitmask) {
			bam_bitplane_draw[by+i*16] |= pixmask;
		} else {
			bam_bitplane_draw[by+i*16] &= ~pixmask;
		}
		bitmask<<=1;
	}
}

void display_flip() {
	bam_bitplane_draw = bam_bitplane_next;
	if (bam_bitplane_next == bam_bitplane_a) {
		bam_bitplane_next=bam_bitplane_b;
	} else {
		bam_bitplane_next=bam_bitplane_a;
	}
}

void display_init() {
	//Initialization and maintaining an image is entirely handled in the task; just start that.
	xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 10, NULL, 1);
}
