#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Bot�o oled 1*/
#define BUT_PIO_OLED1 PIOD
#define BUT_PIO_OLED1_ID ID_PIOD
#define BUT_PIO_OLED1_IDX 28
#define BUT_PIO_OLED1_IDX_MASK (1 << BUT_PIO_OLED1_IDX)

/* Bot�o oled 2*/
#define BUT_PIO_OLED2 PIOC
#define BUT_PIO_OLED2_ID ID_PIOC
#define BUT_PIO_OLED2_IDX 31
#define BUT_PIO_OLED2_IDX_MASK (1 << BUT_PIO_OLED2_IDX)

/* Bot�o oled 3*/
#define BUT_PIO_OLED3 PIOA
#define BUT_PIO_OLED3_ID ID_PIOA
#define BUT_PIO_OLED3_IDX 19
#define BUT_PIO_OLED3_IDX_MASK (1 << BUT_PIO_OLED3_IDX)

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* IN1 */
#define IN1_PIO     PIOD
#define IN1_PIO_ID  ID_PIOD
#define IN1_PIO_PIN 30
#define IN1_PIO_PIN_MASK (1 << IN1_PIO_PIN)

/* IN2 */
#define IN2_PIO     PIOA
#define IN2_PIO_ID  ID_PIOA
#define IN2_PIO_PIN 6
#define IN2_PIO_PIN_MASK (1 << IN2_PIO_PIN)

/* IN3 */
#define IN3_PIO     PIOC
#define IN3_PIO_ID  ID_PIOC
#define IN3_PIO_PIN 19
#define IN3_PIO_PIN_MASK (1 << IN3_PIO_PIN)

/*IN4 */
#define IN4_PIO     PIOA
#define IN4_PIO_ID  ID_PIOA
#define IN4_PIO_PIN 3
#define IN4_PIO_PIN_MASK (1 << IN4_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
SemaphoreHandle_t xSemaphoreRTT;

/** prototypes */
void but_callback(void);
static void BUT_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_oled1_callback(void) {
	char modo = 180;
	xQueueSendFromISR(xQueueModo, &modo, 0);
}

void but_oled2_callback(void) {
	char modo = 90;
	xQueueSendFromISR(xQueueModo, &modo, 0);
}

void but_oled3_callback(void) {
	char modo = 45;
	xQueueSendFromISR(xQueueModo, &modo, 0);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
    gfx_mono_draw_string("Angulo de rotacao", 10, 0, &sysfont);
	gfx_mono_draw_string("0 graus", 35, 16, &sysfont);
	char angulo;
	for (;;)  {
		if(xQueueReceive(xQueueModo, &angulo, 0)){
			gfx_mono_draw_filled_rect(0, 16, 128, 16, GFX_PIXEL_CLR);
			if (angulo == 45){
				gfx_mono_draw_string("45 graus", 35, 16, &sysfont);
				uint32_t steps = 45/0.17578125;
				xQueueSend(xQueueSteps, &steps, 0);
			}
			if (angulo == 90){
				gfx_mono_draw_string("90 graus", 35, 16, &sysfont);
				uint32_t steps = 90/0.17578125;
				xQueueSend(xQueueSteps, &steps, 0);				
			}
			if (angulo == 180){
				gfx_mono_draw_string("180 graus", 35, 16, &sysfont);	
				uint32_t steps = 180/0.17578125;
				xQueueSend(xQueueSteps, &steps, 0);				
			}
		}

	}
}

static void task_motor(void *pvParameters) {
	uint32_t steps = 0;
	for(;;){
		if(xQueueReceive(xQueueSteps, &steps, 0)){
			for (int i = 0; i < steps/4; i++){
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if( xSemaphoreTake(xSemaphoreRTT, 10 / portTICK_PERIOD_MS) == pdTRUE ){
					pio_set(IN1_PIO, IN1_PIO_PIN_MASK);
					pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
					pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
					pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
				}
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if( xSemaphoreTake(xSemaphoreRTT, 10 / portTICK_PERIOD_MS) == pdTRUE ){
					pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
					pio_set(IN2_PIO, IN2_PIO_PIN_MASK);
					pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
					pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
				}
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if( xSemaphoreTake(xSemaphoreRTT, 10 / portTICK_PERIOD_MS) == pdTRUE ){
					pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
					pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
					pio_set(IN3_PIO, IN3_PIO_PIN_MASK);
					pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
				}
				RTT_init(1000, 5, RTT_MR_ALMIEN);
				if( xSemaphoreTake(xSemaphoreRTT, 10 / portTICK_PERIOD_MS) == pdTRUE ){
					pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
					pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
					pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
					pio_set(IN4_PIO, IN4_PIO_PIN_MASK);
				}												
			}
		}
	}
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void io_init(void) {
	
	pmc_enable_periph_clk(IN1_PIO_ID);
	pio_configure(IN1_PIO, PIO_OUTPUT_0, IN1_PIO_PIN_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(IN2_PIO_ID);
	pio_configure(IN2_PIO, PIO_OUTPUT_0, IN2_PIO_PIN_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(IN3_PIO_ID);
	pio_configure(IN3_PIO, PIO_OUTPUT_0, IN3_PIO_PIN_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(IN1_PIO_ID);
	pio_configure(IN4_PIO, PIO_OUTPUT_0, IN4_PIO_PIN_MASK, PIO_DEFAULT);	  
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_OLED1_ID);
	NVIC_SetPriority(BUT_PIO_OLED1_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO_OLED1, PIO_INPUT, BUT_PIO_OLED1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_OLED1, BUT_PIO_OLED1_IDX_MASK, 60);
	pio_enable_interrupt(BUT_PIO_OLED1, BUT_PIO_OLED1_IDX_MASK);
	pio_handler_set(BUT_PIO_OLED1, BUT_PIO_OLED1_ID, BUT_PIO_OLED1_IDX_MASK, PIO_IT_FALL_EDGE , but_oled1_callback);

	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_OLED2_ID);
	NVIC_SetPriority(BUT_PIO_OLED2_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO_OLED2, PIO_INPUT, BUT_PIO_OLED2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_OLED2, BUT_PIO_OLED2_IDX_MASK, 60);
	pio_enable_interrupt(BUT_PIO_OLED2, BUT_PIO_OLED2_IDX_MASK);
	pio_handler_set(BUT_PIO_OLED2, BUT_PIO_OLED2_ID, BUT_PIO_OLED2_IDX_MASK, PIO_IT_FALL_EDGE , but_oled2_callback);
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_OLED3_ID);
	NVIC_SetPriority(BUT_PIO_OLED3_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO_OLED3, PIO_INPUT, BUT_PIO_OLED3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO_OLED3, BUT_PIO_OLED3_IDX_MASK, 60);
	pio_enable_interrupt(BUT_PIO_OLED3, BUT_PIO_OLED3_IDX_MASK);
	pio_handler_set(BUT_PIO_OLED3, BUT_PIO_OLED3_ID, BUT_PIO_OLED3_IDX_MASK, PIO_IT_FALL_EDGE , but_oled3_callback);	
	
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();
	/* Initialize the console uart */
	configure_console();
	xQueueModo = xQueueCreate(32, sizeof(char));
	xQueueSteps = xQueueCreate(32, sizeof(uint32_t));
	xSemaphoreRTT = xSemaphoreCreateBinary();
	
	/* Task modo */
	if (xTaskCreate(task_modo, "modo", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	}
	
	/* Task modo */
	if (xTaskCreate(task_motor, "motor", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create modo task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
