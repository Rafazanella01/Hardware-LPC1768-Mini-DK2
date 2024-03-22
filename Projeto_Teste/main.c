#include "LibCMSIS.h"
#include "LibFreeRTOS.h"
#include "definicoesProjeto.h"
#include <string.h>
#include <stdio.h>
#include <task.h>

static TimerHandle_t blinkTmr;

//================================================================================
//Protótipos de funções

static void blinkCallBack( TimerHandle_t timer );
// Comentei porque estava usando antes, mas agora criei uma task --> static void retornoLed2( TimerHandle_t timer );
void vTaskRetornoLed2(void *pvParameters); // Task
void lerBotao(void *pvParameters); // Task para o botao
TaskHandle_t xTaskHandleBotao;

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
		
		//TaskHandle_t xTaskHandleBotao;

		// Task para o 2° LED
		xTaskCreate(vTaskRetornoLed2, "RetornoLed2_Task", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskHandleBotao);

		// Task para ler o botao
		xTaskCreate(lerBotao, "lerBotao", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

		
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
	

	void lerBotao(void *pvParameters){
		while(1){
		 if (!(GPIO_ReadValue(botao_PORT) & (1 << botao_BIT)))
						vTaskSuspend(xTaskHandleBotao);
						//vTaskResume(xTaskHandleBotao);
		}
				        vTaskDelay(pdMS_TO_TICKS(200));
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
