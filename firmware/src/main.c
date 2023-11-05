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
#define LEDG_PIO      PIOA // LED Verde
#define LEDG_PIO_ID   ID_PIOA
#define LEDG_IDX      6
#define LEDG_IDX_MASK (1 << LEDG_IDX)

#define LEDY_PIO      PIOA // LED Amarelo
#define LEDY_PIO_ID   ID_PIOA
#define LEDY_IDX      3
#define LEDY_IDX_MASK (1 << LEDY_IDX)

#define LEDR_PIO      PIOD // LED Vermelho
#define LEDR_PIO_ID   ID_PIOD
#define LEDR_IDX      31
#define LEDR_IDX_MASK (1 << LEDR_IDX)

#define LEDB_PIO      PIOA // LED Azul
#define LEDB_PIO_ID   ID_PIOA
#define LEDB_IDX      4
#define LEDB_IDX_MASK (1 << LEDB_IDX)


// AFEC
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

#define AFEC_POT2 AFEC1
#define AFEC_POT2_ID ID_AFEC1
#define AFEC_POT2_CHANNEL 6 // Canal do pino PD30

// Botão
#define BUTB_PIO      PIOD // Botão Azul
#define BUTB_PIO_ID   ID_PIOD
#define BUTB_IDX      22
#define BUTB_IDX_MASK (1 << BUTB_IDX)

#define BUTG_PIO PIOD  // Botão Verde
#define BUTG_PIO_ID ID_PIOD
#define BUTG_PIO_IDX	25
#define BUTG_PIO_IDX_MASK (1u << BUTG_PIO_IDX)

#define BUTY_PIO PIOD // Botão Amarelo
#define BUTY_PIO_ID ID_PIOD
#define BUTY_PIO_IDX 24
#define BUTY_PIO_IDX_MASK (1u << BUTY_PIO_IDX)

#define BUTR_PIO PIOA // Botão Vermelho
#define BUTR_PIO_ID ID_PIOA
#define BUTR_PIO_IDX 24
#define BUTR_PIO_IDX_MASK (1u << BUTR_PIO_IDX)

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

SemaphoreHandle_t xGLedSemaphore;
SemaphoreHandle_t xYLedSemaphore;
SemaphoreHandle_t xRLedSemaphore;
SemaphoreHandle_t xBLedSemaphore;
QueueHandle_t xQueueInst;
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

void butB_callback(void);
void butG_callback(void);
void butY_callback(void);
void butR_callback(void);

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
	pmc_enable_periph_clk(LEDG_PIO_ID);
	pmc_enable_periph_clk(LEDY_PIO_ID);
	pmc_enable_periph_clk(LEDR_PIO_ID);
	pmc_enable_periph_clk(LEDB_PIO_ID);

	// Configura Pinos
	pio_configure(LEDG_PIO, PIO_OUTPUT_0, LEDG_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LEDY_PIO, PIO_OUTPUT_0, LEDY_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LEDR_PIO, PIO_OUTPUT_0, LEDR_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LEDB_PIO, PIO_OUTPUT_0, LEDB_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
}

void BUT_init(void){
	// Disativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUTB_PIO_ID);
	pmc_enable_periph_clk(BUTG_PIO_ID);
    pmc_enable_periph_clk(BUTY_PIO_ID);
	pmc_enable_periph_clk(BUTR_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	
	pio_configure(BUTB_PIO, PIO_INPUT, BUTB_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUTB_PIO, BUTB_IDX_MASK, 60);

	pio_configure(BUTG_PIO, PIO_INPUT, BUTG_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUTG_PIO, BUTG_PIO_IDX_MASK, 60);

    pio_configure(BUTY_PIO, PIO_INPUT, BUTY_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUTY_PIO, BUTY_PIO_IDX_MASK, 60);

	pio_configure(BUTR_PIO, PIO_INPUT, BUTR_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUTR_PIO, BUTR_PIO_IDX_MASK, 60);
	
	pio_handler_set(BUTB_PIO, BUTB_PIO_ID, BUTB_IDX_MASK, PIO_IT_FALL_EDGE, butB_callback);

	pio_handler_set(BUTG_PIO, BUTG_PIO_ID, BUTG_PIO_IDX_MASK, PIO_IT_FALL_EDGE, butG_callback);

    pio_handler_set(BUTY_PIO, BUTY_PIO_ID, BUTY_PIO_IDX_MASK, PIO_IT_FALL_EDGE, butY_callback);

	pio_handler_set(BUTR_PIO, BUTR_PIO_ID, BUTR_PIO_IDX_MASK, PIO_IT_FALL_EDGE, butR_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	
	pio_enable_interrupt(BUTB_PIO, BUTB_IDX_MASK);
	pio_get_interrupt_status(BUTB_PIO);
	
	pio_enable_interrupt(BUTG_PIO, BUTG_PIO_IDX_MASK);
	pio_get_interrupt_status(BUTG_PIO);

    pio_enable_interrupt(BUTY_PIO, BUTY_PIO_IDX_MASK);
	pio_get_interrupt_status(BUTY_PIO);

	pio_enable_interrupt(BUTR_PIO, BUTR_PIO_IDX_MASK);
	pio_get_interrupt_status(BUTR_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)

	NVIC_EnableIRQ(BUTB_PIO_ID);
	NVIC_SetPriority(BUTB_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUTG_PIO_ID);
	NVIC_SetPriority(BUTG_PIO_ID, 4);

    NVIC_EnableIRQ(BUTY_PIO_ID);
	NVIC_SetPriority(BUTY_PIO_ID, 4);

	NVIC_EnableIRQ(BUTR_PIO_ID);
	NVIC_SetPriority(BUTR_PIO_ID, 4);
}

void send_package(char package){
	while(!usart_is_tx_ready(USART_COM)) {
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	usart_write(USART_COM, package);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		// .charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		// .stopbits = CONF_UART_STOP_BITS,
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
	uint32_t i = 0;

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
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEEngHawaii", 100);
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

void butB_callback(void){
	// if (pio_get(BUTB_PIO, PIO_INPUT, BUTB_IDX_MASK)){
		Instruction inst;
		inst.id = 4;
		inst.value = 1;
		xQueueSendFromISR(xQueueInst, &inst, 0);

		xSemaphoreGiveFromISR(xBLedSemaphore, 0);
	// }
}

void butG_callback(void){
	// if (pio_get(BUTG_PIO, PIO_INPUT, BUTG_PIO_IDX_MASK)){
	Instruction inst;
	inst.id = 1;
	inst.value = 1;
	xSemaphoreGiveFromISR(xGLedSemaphore, 0);
	
	xQueueSendFromISR(xQueueInst, &inst, 0);	
	// }
}

void butY_callback(void) {
	// if (pio_get(BUTY_PIO, PIO_INPUT, BUTY_PIO_IDX_MASK)){
		Instruction inst;
		inst.id = 2;
		inst.value = 1;
		xQueueSendFromISR(xQueueInst, &inst, 0);

		xSemaphoreGiveFromISR(xYLedSemaphore, 0);
	// }
}

void butR_callback(void){ 
	// if (pio_get(BUTR_PIO, PIO_INPUT, BUTR_PIO_IDX_MASK)){
		Instruction inst;
		inst.id = 3;
		inst.value = 1;
		xQueueSendFromISR(xQueueInst, &inst, 0);

		xSemaphoreGiveFromISR(xRLedSemaphore, 0);
	// }
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
				afec.id = 5;
				afec.value = vol;
				xQueueSendFromISR(xQueueInst, &afec, 0);
			}
			oldmsg = msg;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void task_bluetooth(void) {

	config_usart0();
	hc05_init();

	io_init();
	
	char hs = 'A';
	char eof = 'X';
	Instruction msg;
	char Handshake = 0;
	char sleep = 1;

	while(1) {
		while (Handshake != 'A'){
				send_package(hs);
				
				usart_read(USART_COM, &Handshake);
			}
		if (xQueueReceive(xQueueInst, &msg, (TickType_t) 0)){
				char msg_id = msg.id;
				char msg_value = msg.value;
				
				send_package(msg_id);
				
				send_package(msg_value);

				send_package(eof);
		}
		if (xSemaphoreTake(xBLedSemaphore, 0)) {
			sleep += 1;
			if (!(sleep % 2)) {
			pio_set(LEDB_PIO, LEDB_IDX_MASK);
			} else {
			pio_clear(LEDB_PIO, LEDB_IDX_MASK);
			Handshake = 0;
			}
		}
		
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
		
}

void task_led(void) {
	pio_clear(LEDG_PIO, LEDG_IDX_MASK);
	pio_clear(LEDY_PIO, LEDY_IDX_MASK);
	pio_clear(LEDR_PIO, LEDR_IDX_MASK);
	pio_clear(LEDB_PIO, LEDB_IDX_MASK);
	while(1) {
		if (xSemaphoreTake(xGLedSemaphore, 0)) {
			pio_set(LEDG_PIO, LEDG_IDX_MASK);
			vTaskDelay(500 / portTICK_PERIOD_MS);
			pio_clear(LEDG_PIO, LEDG_IDX_MASK);
		}
		if (xSemaphoreTake(xYLedSemaphore, 0)) {
			pio_set(LEDY_PIO, LEDY_IDX_MASK);
			vTaskDelay(500 / portTICK_PERIOD_MS);
			pio_clear(LEDY_PIO, LEDY_IDX_MASK);
		}
		if (xSemaphoreTake(xRLedSemaphore, 0)) {
			pio_set(LEDR_PIO, LEDR_IDX_MASK);
			vTaskDelay(500 / portTICK_PERIOD_MS);
			pio_clear(LEDR_PIO, LEDR_IDX_MASK);
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	xQueueInst = xQueueCreate(4, sizeof(Instruction));
	xQueueAFEC =  xQueueCreate(1, sizeof(int));
	xGLedSemaphore = xSemaphoreCreateBinary();
	xYLedSemaphore = xSemaphoreCreateBinary();
	xRLedSemaphore = xSemaphoreCreateBinary();
	xBLedSemaphore = xSemaphoreCreateBinary();


	sysclk_init();
	board_init();
	BUT_init();

	configure_console();

	/* Create task to make led blink */
	if (xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create bluetooth task\r\n");
	}

	if (xTaskCreate(task_process, "PROC", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create process task\r\n");
	}

	if (xTaskCreate(task_led, "LED", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create led task\r\n");
	}
	// xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	// xTaskCreate(task_process, "PROC", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	// xTaskCreate(task_led, "LED", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
