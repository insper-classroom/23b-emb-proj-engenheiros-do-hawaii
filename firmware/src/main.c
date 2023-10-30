/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO      PIOA
#define LED_PIO_ID   ID_PIOA
#define LED_IDX      6
#define LED_IDX_MASK (1 << LED_IDX)

// AFEC
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

#define AFEC_POT2 AFEC1
#define AFEC_POT2_ID ID_AFEC1
#define AFEC_POT2_CHANNEL 6 // Canal do pino PD30

// Botão
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX      11
#define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT1_PIO PIOD  // Botão 1
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX	28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO PIOC // Botão 2
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO PIOA // Botão 3
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

QueueHandle_t xQueueLMFAO;
QueueHandle_t xQueueAFEC;
TimerHandle_t xTimer;

struct {
	char id;
	char value;
}typedef Instruction;

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void but0_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);

	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
}

void BUT_init(void){
	// Disativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
    pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_IDX_MASK, 60);

	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);

    pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);

	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_IDX_MASK, PIO_IT_EDGE, but0_callback);

	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, but1_callback);

    pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, but2_callback);

	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, but3_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_get_interrupt_status(BUT_PIO);
	
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);

    pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);

	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)

	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);

    NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}


void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEHawaii", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PINVSCO", 100);
}

void vTimerCallback(TimerHandle_t xTimer) {
	
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
	
}

static void AFEC_pot_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	
	int adc;
	adc = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	xQueueSendFromISR(xQueueAFEC, &adc, &xHigherPriorityTaskWoken);
	
}

void but0_callback(void){
	if (pio_get(BUT_PIO, PIO_INPUT, BUT_IDX_MASK)){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		Instruction inst;
		inst.id = 8;
		inst.value = 1;
		xQueueSendFromISR(xQueueLMFAO, &inst, xHigherPriorityTaskWoken);
	}
}

void but1_callback(void){
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	Instruction inst;
	inst.id = 1;
	inst.value = 1;
	xQueueSendFromISR(xQueueLMFAO, &inst, xHigherPriorityTaskWoken);	
	}
}

void but2_callback(void) {
	if (pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		Instruction inst;
		inst.id = 2;
		inst.value = 1;
		xQueueSendFromISR(xQueueLMFAO, &inst, xHigherPriorityTaskWoken);
	}
}

void but3_callback(void){ 
	if (pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		Instruction inst;
		inst.id = 4;
		inst.value = 1;
		xQueueSendFromISR(xQueueLMFAO, &inst, xHigherPriorityTaskWoken);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void task_process(void){
	
	xTimer = xTimerCreate(
	"Timer",
	100,
	pdTRUE,
	(void *)0,
	/* Timer callback */
	vTimerCallback);
	xTimerStart(xTimer, 0);
	
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);
	int oldmsg = 0;
	int msg;
	
	while(1){
		if(xQueueReceive(xQueueAFEC, &msg, (TickType_t) 0)){
			if ((msg - oldmsg) > 10){
				char vol = (msg*100)/4095;
				Instruction afec;
				afec.id = 16;
				afec.value = vol;
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xQueueSendFromISR(xQueueLMFAO, &afec, &xHigherPriorityTaskWoken);
			}
			oldmsg = msg;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void task_bluetooth(void) {
	char Handshake = 0;
	uint32_t var = 0;
	printf("Task Bluetooth started \n");
	
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();

	io_init();
	
	char hs = 'A';
	char eof = 'X';
	Instruction msg;

	while(1) {
		while (Handshake != 'A'){
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, hs);
			
			usart_read(USART_COM, &Handshake);
			printf("Enviando Handshake\n");
		}
		if (xQueueReceive(xQueueLMFAO, &msg, (TickType_t) 0)){
			char id = msg.id;
			char value = msg.value;
			
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, id);
			
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, value);

			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, eof);
			
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
		
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	BUT_init();

	configure_console();
	xQueueLMFAO = xQueueCreate(4, sizeof(Instruction));
	xQueueAFEC =  xQueueCreate(1, sizeof(int));

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	xTaskCreate(task_process, "PROC", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
