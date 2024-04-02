#include "LibCMSIS.h"
#include "LibFreeRTOS.h"
#include "definicoesProjeto.h"
#include <string.h>
#include <stdio.h>
#include <task.h>
#include "LibNetwork.h"

#define TAMANHO 50

static TimerHandle_t blinkTmr;

static TaskHandle_t xTask;

static SemaphoreHandle_t mutexADC;

static SemaphoreHandle_t Potenciometro;

static uint8_t ip[4] = {192, 168, 53, 216};
static uint8_t mask[4] = {255, 255, 255, 0};
static uint8_t gateway[4] = {192, 168, 53, 254};
static uint8_t dns[4] = {8, 8, 8, 8};
uint8_t ucMACAddress[6] = {0X00, 0X18, 0X80, 0X01, 0X02, 0X03};

//================================================================================
//Protótipos de funções
static void blinkCallBack( TimerHandle_t timer );
void vTaskRetornoLed2(void *pvParameters); // Task
void lerBotaoLed(void *pvParameters); // Task para o botao
void recebeEnviaSerial(void *pvParameters); // Task para receber o que foi enviado pra fila

// handle para suspender o led que for passado a handle
TaskHandle_t xTaskHandleBotao;

//QueueHandle_t xMessageQueue;

// Handles dos semáforos
SemaphoreHandle_t xSemaphoreBotao1;
SemaphoreHandle_t xSemaphoreBotao2;

void leBotoesMensagens(void *pvParameters); // Task para os botoes que exibirao uma mensagem

// Prototipo da função o canal é o 3
uint16_t leCanalAD( unsigned char canal );

// task pro potenciometro
void potenciometro(void *pvParameters);

void mensagemUART(char msg[]);

static void vUDPServer( void *pvParameters );

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
	
		//configura P0.26 como AD0.3 (medição de tensão da bateria)
		pinsel.Portnum= PINSEL_PORT_0;		    
		pinsel.Pinmode= PINSEL_PINMODE_TRISTATE;	    
		pinsel.Pinnum= PINSEL_PIN_26;
		pinsel.Funcnum= 1;
		PINSEL_ConfigPin( &pinsel );  

		//inicializa o conversor AD do LPC1768
		ADC_Init( LPC_ADC, 100000 );
		
		//==============================================             
		
		// Fila
		//xMessageQueue = xQueueCreate(50, sizeof(char[TAMANHO]));
		
		// Cria o mutex que vai ser feito o take na função leCanalAD
		mutexADC = xSemaphoreCreateMutex();
		
		Potenciometro = xSemaphoreCreateMutex();
						
		// timer do 1° LED
		blinkTmr= xTimerCreate( "blinkTmr", pdMS_TO_TICKS(100), pdTRUE, 0, blinkCallBack);		
		xTimerStart( blinkTmr, 0 );   
		
		// Task para o 2° LED
		xTaskCreate(vTaskRetornoLed2, "RetornoLed2_Task", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskHandleBotao);

		// Task para ler o botao que apaga o led
		xTaskCreate(lerBotaoLed, "lerBotaoLed", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

		// Faz a leitura dos botões e envia um sinal para a função recebeEnviaSerial
		xTaskCreate(leBotoesMensagens, "leBotoesMensagens", 40, NULL, 1, NULL);
		
		// Recebe as mensagens da fila e imprime na tela
		xTaskCreate(recebeEnviaSerial, "recebeEnviaSerial", configMINIMAL_STACK_SIZE, NULL, 1, &xTask);
		
		xTaskCreate(potenciometro, "potenciometro", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		
		xTaskCreate(vUDPServer, "teste", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		
		FreeRTOS_IPInit( ip, mask, gateway, dns, ucMACAddress );

		vTaskStartScheduler();
		
}
//================================================================================

void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent ){
	}

	uint16_t leCanalAD( unsigned char canal ){    
	
	uint16_t valorAD;

	//obtém o mutex para acesso ao ADC
	xSemaphoreTake( mutexADC, portMAX_DELAY );   

	//habilita o canal desejado 
	ADC_ChannelCmd( LPC_ADC, canal, ENABLE );   

	//Inicia uma conversão
	ADC_StartCmd( LPC_ADC, ADC_START_NOW );
	
	//Espera a conversão terminar
	while( ADC_ChannelGetStatus(LPC_ADC, canal, ADC_DATA_DONE) == RESET );

	//lê o resultado da conversão
	valorAD= ADC_ChannelGetData ( LPC_ADC, canal );    

	//desabilita o canal que foi habilitado para realizar a conversão
	ADC_ChannelCmd( LPC_ADC, canal, DISABLE );
 
	//delay_us( 10 );

	//devolve o mutex para acesso ao ADC
	xSemaphoreGive( mutexADC );
	
	return valorAD;    

}

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
	
void leBotoesMensagens(void *pvParameters) {
//xSemaphoreBotao1 = xSemaphoreCreateBinary();
xSemaphoreBotao2 = xSemaphoreCreateBinary();

TickType_t ultimoTempoBotao1 = 0;
TickType_t ultimoTempoBotao2 = 0;

BaseType_t botao1Enviado = pdFALSE;
BaseType_t botao2Enviado = pdFALSE;

while(1) {
		if (!(GPIO_ReadValue(botao_1_PORT) & (1 << botao_1_BIT))) {
				if (xTaskGetTickCount() - ultimoTempoBotao1 > pdMS_TO_TICKS(500)) {
						if (!botao1Enviado) {
								//xSemaphoreGive(xSemaphoreBotao1);
								xTaskNotifyGive(xTask);
								// UART_Send(LPC_UART0, (uint8_t *)mensagem1, strlen(mensagem1), BLOCKING);
								// xQueueSend(xMessageQueue, mensagem1, 0);
								botao1Enviado = pdTRUE;
						}
						ultimoTempoBotao1 = xTaskGetTickCount();
				}
		} else {
				ultimoTempoBotao1 = 0; 
				botao1Enviado = pdFALSE;        }

		if (!(GPIO_ReadValue(botao_2_PORT) & (1 << botao_2_BIT))) {
				if (xTaskGetTickCount() - ultimoTempoBotao2 > pdMS_TO_TICKS(500)) {
						if (!botao2Enviado) {
								xSemaphoreGive(xSemaphoreBotao2);
								// UART_Send(LPC_UART0, (uint8_t *)mensagem2, strlen(mensagem2), BLOCKING);
								// xQueueSend(xMessageQueue, mensagem2, 0);
								botao2Enviado = pdTRUE;
						}
						ultimoTempoBotao2 = xTaskGetTickCount();
				}
		} else {
				ultimoTempoBotao2 = 0; 
				botao2Enviado = pdFALSE; 
		}
}
}

void mensagemUART(char msg[]){
	 xSemaphoreTake(Potenciometro, portMAX_DELAY);
	 UART_Send(LPC_UART0, (uint8_t *)msg, strlen(msg), BLOCKING);	
	 xSemaphoreGive(Potenciometro);



	
}

void recebeEnviaSerial(void *pvParameters){
		const char * mensagem1 = "\n Botao 1 do rafa \r\n";
		const char * mensagem2 = "\n O Rafael Zanella eh lindo \r\n";
		//char mensagem[TAMANHO];
		
		while(1){		
			if(ulTaskNotifyTake(pdTRUE, 0)){
				//if (xQueueReceive(xMessageQueue, mensagem, portMAX_DELAY) == pdPASS) {
				mensagemUART((char *)mensagem1);
				vTaskDelay(pdMS_TO_TICKS(200));
			//}          
			} 
			
			if(xSemaphoreTake(xSemaphoreBotao2, 0) == pdPASS){
				mensagemUART((char *)mensagem2);
				vTaskDelay(pdMS_TO_TICKS(200));					
			}
			vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void potenciometro(void *pvParameters){
	 char buffer[20];
    while (1) {
        // Lê o valor do potenciômetro
        uint16_t valor = leCanalAD(3);
        
        
        // Converte o valor lido para string
        sprintf(buffer, "%u\r\n", valor);
        
        // Envia o valor via UART
				mensagemUART((char *)buffer);
                			
        // Aguarda um tempo antes de realizar a próxima leitura
        vTaskDelay(pdMS_TO_TICKS(150)); 
    }
}

static void vUDPServer( void *pvParameters ){    
    static const TickType_t 
        xReceiveTimeOut = portMAX_DELAY;
    struct freertos_sockaddr         
        xBindAddress, xSourceAddress;
    static uint8_t
        udpServerRxBuffer[1024],
        aesBuf[1024];
    BaseType_t 
        lBytesReceived;         
   
		Socket_t xListeningSocket;
   
    /* Cria o socket. */
	
		xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );
		
	  
    
    //
    FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof(xReceiveTimeOut) );
    
    //    
    xBindAddress.sin_port= FreeRTOS_htons( 5000 );
    
    /* Bind the socket to the port that the client RTOS task will send to. */
    FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );   
    
    
    for(;;){        
        
        //====================================================================        
        
        memset( aesBuf, 0, sizeof(aesBuf) );          
        
        lBytesReceived= FreeRTOS_recvfrom( xListeningSocket, aesBuf, sizeof(aesBuf), 0, &xSourceAddress, (socklen_t *)sizeof(xSourceAddress) );        
                                
            
				//====================================================================
				
				if( strlen((char *)aesBuf) ){
				
						//====================================================================
						
						//                 
						
						/* Num teste alternando os comandos de abrir e fechar pista a cada 100ms,
						não retorna mais desta função após processar alguns comandos. Provavelmente,
						por não conseguir mais alocar memória para o envio, pois o timeout default do envio
						é portMAX_DELAY. */
						FreeRTOS_sendto( xListeningSocket, aesBuf, lBytesReceived, 0, &xSourceAddress, sizeof(xSourceAddress) );  

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
