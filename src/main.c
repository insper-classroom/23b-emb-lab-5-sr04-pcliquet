#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1u << BUT_PIO_PIN)

#define TRIG_PIO	 PIOA
#define TRIG_PIO_ID  ID_PIOA
#define TRIG_PIO_PIN 4u
#define TRIG_PIO_PIN_MASK (1u << TRIG_PIO_PIN)

#define ECHO_PIO	 PIOA
#define ECHO_PIO_ID  ID_PIOA
#define ECHO_PIO_PIN 3u
#define ECHO_PIO_PIN_MASK (1u << ECHO_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_TEMPO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_TEMPO_STACK_PRIORITY            (tskIDLE_PRIORITY)

typedef struct {
  uint value;
} distData;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void trig_callback(void);
void echo_callback(void);
void RTT_Handler(void);
static void BUT_init(void);
static void sensor_init(void);
static void send_trigger(void);
volatile int 	flag_rtt;





/************************************************************************/
/* Semaforos                                           */
/************************************************************************/

SemaphoreHandle_t xSemaphoreTempo;

/************************************************************************/
/*Filas								                                     */
/************************************************************************/

QueueHandle_t xQueueTempoSensor;

QueueHandle_t xQueueDistancia;

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

void but_callback(void) {
	send_trigger();
}


void echo_callback(void){
	int ul_previous_time;
	if(pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)){
		RTT_init(8000, 0, 0 );	
	}
	else{
		ul_previous_time = rtt_read_timer_value(RTT);
		
		xQueueSendFromISR(xQueueTempoSensor, (void *)&ul_previous_time, 0);
	}
	
	
}

void send_trigger(void){
	pio_set(TRIG_PIO, TRIG_PIO_PIN_MASK);
	delay_us(10);
	pio_clear(TRIG_PIO, TRIG_PIO_PIN_MASK);

}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		flag_rtt = 1;
	}
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
  gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
 
	int msg;
	for (;;)  {
		if(xQueueReceive(xQueueTempoSensor, &msg, (TickType_t) 0)){
// 			float tempo = msg;
// 			printf("teste: %d \n",ul_previous_time);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			float t = (msg)/8000.0;
			printf("%f \n",t);
			float d = t*340.42/2;
			char str[20];
			sprintf(str, "d: %f m", d);
			gfx_mono_draw_string(str, 10, 8, &sysfont);
			xSemaphoreGiveFromISR(xSemaphoreTempo,xHigherPriorityTaskWoken);
		}

	}
}

static void task_tempo(void *pvParameters){
	float ul_previous_time;
	
	while(1){
		if (xSemaphoreTake(xSemaphoreTempo, 1000)){
			send_trigger();
			
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



static void BUT_init(void) {
	/* configura prioridae */


	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT_PIO);
	
	NVIC_EnableIRQ(BUT_PIO_ID);
		NVIC_SetPriority(BUT_PIO_ID, 4);
}

static void sensor_init(void){
	pmc_enable_periph_clk(TRIG_PIO_ID);
		pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_PIN_MASK, PIO_DEFAULT);
	
	
	
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE, echo_callback);
	
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_get_interrupt_status(ECHO_PIO);

	
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	
	 xSemaphoreTempo = xSemaphoreCreateBinary();
	  if (xSemaphoreTempo == NULL){
		  printf("falha em criar o semaforo \n");
	  }
	
	
	xQueueTempoSensor = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueTempoSensor == NULL){
		printf("falha em criar a queue Tempo do sensor \n");
	}
	
	xQueueDistancia = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueDistancia == NULL){
		printf("falha em criar a queue Distancia \n");
	}
	
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	BUT_init();
	sensor_init();
	
	gfx_mono_draw_string("susi", 10, 8, &sysfont);
	/* Initialize the console uart */
	configure_console();
	 

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create OLED task\r\n");
	}
	
	if (xTaskCreate(task_tempo, "tempo", TASK_TEMPO_STACK_SIZE, NULL, TASK_TEMPO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create TEMPO task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
