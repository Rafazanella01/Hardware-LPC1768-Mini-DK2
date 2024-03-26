#include "LibCMSIS.h"
#include "LibFreeRTOS.h"
#include "definicoesProjeto.h"
#include <string.h>
#include <stdio.h>
#include <task.h>

#define tamanho 50

static TimerHandle_t blinkTmr;

//================================================================================
//Protótipos de funções

static void blinkCallBack( TimerHandle_t timer );
void vTaskRetornoLed2(void *pvParameters); // Task
void lerBotaoLed(void *pvParameters); // Task para o botao
void recebeFila(void *pvParameters); // Task para receber o que foi enviado pra fila

// handle para suspender o led que for passado a handle
TaskHandle_t xTaskHandleBotao;

QueueHandle_t xMessageQueue;

void mensagemKeys(void *pvParameters); // Task para os botoes que exibirao uma mensagem
//================================================================================

int main( void ){
    UART_CFG_Type
        uartCfg;
    UART_FIFO_CFG_Type
        uartFifo;
    PINSEL_CFG_Type
        pinsel;       
    
    //
    if( LPC_SC->PLL0CFG == 0 )
        SystemInit();
    SystemCoreClockUpdate();
    NVIC_SetPriorityGrouping( 2 );
    
    //=============================================================================
    
    //RUN LED como saída
    GPIO_SetDir( RUNLED_PORT, 1 << RUNLED_BIT, 1 );
    GPIO_SetDir(led2_PORT, 1 << led2_BIT, 1);
   
    //==============================================
    //UART0 (USB_Serial)
    
    UART_FIFOConfigStructInit( &uartFifo );
    
    UART_ConfigStructInit( &uartCfg );
    
    UART_Init( LPC_UART0, &uartCfg );
    
    UART_TxCmd( LPC_UART0, ENABLE );

    //TXD0
    pinsel.Funcnum= 1;
    pinsel.Portnum= 0;
    pinsel.Pinnum= 2;
    PINSEL_ConfigPin( &pinsel );
    
    //RXD0
    pinsel.Funcnum= 1;
    pinsel.Portnum= 0;
    pinsel.Pinnum= 3;
    PINSEL_ConfigPin( &pinsel );    
  
    //==============================================             
    
		// timer do 1° LED
		blinkTmr= xTimerCreate( "blinkTmr", pdMS_TO_TICKS(100), pdTRUE, 0, blinkCallBack);		
    xTimerStart( blinkTmr, 0 );   
		
		// Task para o 2° LED
		xTaskCreate(vTaskRetornoLed2, "RetornoLed2_Task", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskHandleBotao);

		// Task para ler o botao
		xTaskCreate(lerBotaoLed, "lerBotaoLed", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

		// Faz a leitura das mensagens e envia para a fila
		xTaskCreate(mensagemKeys, "mensagemKeys", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		
		// Recebe as mensagens da fila e imprime na tela
		xTaskCreate(recebeFila, "recebeFila", configMINIMAL_STACK_SIZE, NULL,/*tskIDLE_PRIORITY*/ 2, NULL);
		
		xMessageQueue = xQueueCreate(50, sizeof(char[tamanho]));

    vTaskStartScheduler();
}
//================================================================================

static void blinkCallBack( TimerHandle_t timer ){
    if( GPIO_ReadValue(RUNLED_PORT) & (1 << RUNLED_BIT) )
        GPIO_ClearValue( RUNLED_PORT, 1 << RUNLED_BIT );
    else
        GPIO_SetValue( RUNLED_PORT, 1 << RUNLED_BIT );   
	
}

void vTaskRetornoLed2(void *pvParameters) { 
    while (1) {
        if (GPIO_ReadValue(led2_PORT) & (1 << led2_BIT))
            GPIO_ClearValue(led2_PORT, 1 << led2_BIT);
        else
            GPIO_SetValue(led2_PORT, 1 << led2_BIT);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
	
void lerBotaoLed(void *pvParameters){
		while(1){
		 if (!(GPIO_ReadValue(botao_PORT) & (1 << botao_BIT)))
						vTaskSuspend(xTaskHandleBotao);
						//vTaskResume(xTaskHandleBotao);
		 }
				        vTaskDelay(pdMS_TO_TICKS(200));
} 
	
void mensagemKeys(void *pvParameters){
		const char * mensagem1 = "\n Botao 1 do rafa \r\n";
		const char * mensagem2 = "\n O Rafael Zanella eh lindo \r\n";
		
		while(1){
			 if (!(GPIO_ReadValue(botao_1_PORT) & (1 << botao_1_BIT))){
				 
				// UART_Send(LPC_UART0, (uint8_t *)mensagem1, strlen(mensagem1), BLOCKING);
				 xQueueSend(xMessageQueue, mensagem1, 0);
				 vTaskDelay(pdMS_TO_TICKS(200));
				 
				}else if(!(GPIO_ReadValue(botao_2_PORT) & (1 << botao_2_BIT))){
				
				 //UART_Send(LPC_UART0, (uint8_t *)mensagem2, strlen(mensagem2), BLOCKING);
				 xQueueSend(xMessageQueue, mensagem2, 0);
				 vTaskDelay(pdMS_TO_TICKS(200));
				}
		}
}

void recebeFila(void *pvParameters){
	char mensagem[tamanho];
		while(1){		
			if (xQueueReceive(xMessageQueue, mensagem, portMAX_DELAY) == pdPASS) {
          UART_Send(LPC_UART0, (uint8_t *)mensagem, strlen(mensagem), BLOCKING);
					vTaskDelay(pdMS_TO_TICKS(200));
			}
		}
}

	
//================================================================================
/*No momento não está sendo usada, pois o kernel está sendo compilado
sem a opção configASSERT */


void vAssertCalled( const char *pcFile, uint32_t ulLine ){
    char strAux[32];
    
    sprintf( strAux, "F:%s, L:%u\r\n", (char *)*pcFile, ulLine );
    UART_Send( LPC_UART0, (uint8_t *)strAux, strlen(strAux), BLOCKING );
}

//================================================================================

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ){ 

    taskDISABLE_INTERRUPTS();
    
    while( 1 ){
        UART_Send( LPC_UART0, (uint8_t *)pcTaskName, strlen((char *)pcTaskName), BLOCKING );
        UART_Send( LPC_UART0, "\r\n", 2, BLOCKING );//quebra de linha p/ facilitar a leitura no terminal serial
        
        if( GPIO_ReadValue(RUNLED_PORT) & (1 << RUNLED_BIT) )
            GPIO_ClearValue( RUNLED_PORT, 1 << RUNLED_BIT );
        else
            GPIO_SetValue( RUNLED_PORT, 1 << RUNLED_BIT );
        
        //delay_ms( 100 );
    }
    
}
