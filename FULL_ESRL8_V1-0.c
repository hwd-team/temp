/*
 * HWD_Bridge.c
 *
 * Created: 16/10/2014 18:42:52
 *  Author: Webert
 */ 


//#define F_CPU 11059200UL
#define	F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include <avr/interrupt.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "LightweightRingBuff.h"
#include "rsRingBuffer.h"
#include "ids_list.h"

#define BAUD_232 115200
#define my232UBRR F_CPU/16/BAUD_232-1

#define BAUD_485 115200
#define my485UBRR F_CPU/16/BAUD_485-1

#define ACK			0x06
#define NAK			0x15
#define TO_BRIDGE	0xA0
#define TO_GO		0xA1
#define BRIDGE_ADD	0x01
#define BRIDGE_REM	0x00

#define ERR				0xF2
#define ERR_CMD1		0xF0
#define ERR_CMD2		0xF1
#define	ERR_CHECKSUM	0xEF

#define VIRTUAL_CHECKSUM	0xAA

#define HWD_NET		PC2


int kk = 0, ll = 0, rele_cmd = 0xFF, rele_out[8] = { 1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7};
#define DEBUG		1

uint8_t qde_blink;
uint8_t VNET_backup;

uint16_t keep_alive;

id_list IDs;
bool rs485_Timedout;
char mem_rx485Str[10];
uint8_t	modulo_status_in[0xFF];
uint8_t	modulo_status_out[0xFF];

// Flag habilitar varredura
bool scan_status;

// Porta Serial 232
bool reception232Complete;
volatile RingBuff_t *tx232RingBuff, tx232VRingBuff; // RING BUFFER QUE ARMAZENA STRINGS A SEREM ENVIADAS
// Recepcao
uint8_t str232[10];
uint8_t *rx232Str;
uint8_t *rx232StrPtr;

// Fila da porta serial
rsRingBuffer_Data_t data_in[ELEMENT_SIZE];
rsRingBuffer_Data_t data_out[ELEMENT_SIZE];

typedef struct
{
	uint8_t msg[10];
	uint8_t count;
} last_msg;

last_msg last_sent_msg;

rsRingBuffer_t rsBuffer;
rsRingBuffer_t *rsBufferPtr;


// Porta Serial 485
bool reception485Complete;
volatile RingBuff_t *tx485RingBuff, tx485VRingBuff; // RING BUFFER QUE ARMAZENA STRINGS A SEREM ENVIADAS
// Recepcao
uint8_t str485[10];
uint8_t *rx485Str;
uint8_t *rx485StrPtr;


void RingBuffAddChar(RingBuff_t* const buffer, uint8_t* str, uint8_t tam);

// FUNÇÕES PORTA SERIAL 232
void USART_232_init(unsigned int baud_rate);
void USART_232_transmit();
void decode232Str();
void limpa_232Str();


// FUNÇÕES PORTA SERIAL 485
void USART_485_init(unsigned int baud_rate);
void USART_485_transmit();
void decode485Str();
void limpa_485Str();
void MakeStrCmd(char ID, char tipo);

void Make485Cmd(rsRingBuffer_Data_t *data, char ID, char tipo, char CMD1, char CMD2);
void send485Cmd(char ID, char tipo, char CMD1, char CMD2);

//void Make232Cmd(rsRingBuffer_Data_t *data, char CMD1, char CMD2, char PARAM_1, char PARAM_2, char PARAM_3);
void Make232Cmd(rsRingBuffer_Data_t *data, char CMD_ID, char CMD_TYPE, char CMD1, char CMD2, char CMD3, char CMD4);

void keep_alive_Start_Timeout();
void keep_alive_Stop_Timeout();

void REG16_WriteOCR1A(unsigned int i);
void rs485_Start_Timeout();
void rs485_Stop_Timeout();

void REG16_WriteOCR3C(unsigned int i);
void PWM_Start_Timeout();
void PWM_Stop_Timeout();

int main(void)
{
	uint8_t i;
	id_item *modulo;
	
	// Timer0
	// Configura Timer1 para o modo CTC
	TCCR0 |= (1 << WGM01);
	// Habilita interrupcao CTC para o comparador A
	TIMSK |= (1 << OCIE0);
	// Define o limite de contagem do CTC. Configuracao de 1cs = 10ms a um clock de 14,7456 MHz e prescaler de 1024
	OCR0 = 0x8F;
	keep_alive_Stop_Timeout();
	
	// Timer1
	// Configura Timer1 para o modo CTC
	TCCR1B |= (1 << WGM12);
	// Habilita interrupcao CTC para o comparador A
	TIMSK |= (1 << OCIE1A);
	// Define o limite de contagem do CTC. Configuracao de 1,5ms a um clock de 14,7456 MHz e prescaler de 1024
	REG16_WriteOCR1A(20);
	rs485_Stop_Timeout();
	
	// Timer3
	// Configura Timer3 para o modo CTC
	TCCR3B |= (1 << WGM32);
	// Habilita interrupcao CTC para o comparador C
	ETIMSK |= (1 << OCIE3C);
	// Define o limite de contagem do CTC. Configuracao de 500ms a um clock de 14,7456 MHz e prescaler de 1024
	//REG16_WriteOCR3C(1439);
	REG16_WriteOCR3C(14399);
	PWM_Stop_Timeout();
	
	// Iniciar String de recepção da porta serial 232
	rx232Str = str232;		// Inicialização do ponteiro
	rx232StrPtr = str232;
	reception232Complete = false;
	
	// Iniciar String de recepção da porta serial 485
	rx485Str = str485;		// Inicialização do ponteiro
	rx485StrPtr = str485;
	reception485Complete = false;
	
	rsBufferPtr = &rsBuffer;
	rsRingBuffer_Init(rsBufferPtr);
	
	for (i = 0; i < 0xFF; i++) {
		modulo_status_in[i] = 0x00;
		modulo_status_out[i] = 0x00;
	}
	
	cli();
	
	// Habilitar RS485_EN  e HWD-NET como saida
	DDRC |= (1 << PC0) | (1 << PC2);
	
	// Setar MAX485 como recepcao
	PORTC &= ~(1 << PC0);
	
	// Setar HWD-NET
	//PORTC |= (1 << HWD_NET);
	
	// PORTA SERIAL 232
	tx232RingBuff = &tx232VRingBuff;
	RingBuffer_InitBuffer((RingBuff_t* const) tx232RingBuff);
	
	// PORTA SERIAL 485
	tx485RingBuff = &tx485VRingBuff;
	RingBuffer_InitBuffer((RingBuff_t* const) tx485RingBuff);
	
	id_init(&IDs);
	for (i = 1; i <= 30; i++) {
		if (eeprom_read_byte((uint8_t *) (uint16_t) i) == 1) {
			if (id_insert(&IDs, i, (uint8_t) 1) == 0x15) {
				// Erro ao inserir ID
			}
		}
	}

	USART_232_init(my232UBRR);
	USART_485_init(my485UBRR);
	
	sei();
	
	_delay_ms(100);
	
	rs485_Timedout = true;
	
	// Habilitar/Desabilitar Varredura inicialmente
	if (id_getCount(&IDs) > 0)	{ scan_status = true;  }
	else						{ scan_status = false; }
	
	//PWM_Start_Timeout();
	
    while(1)
    {
        if (reception232Complete) {
	        decode232Str();
	        reception232Complete = false;
        } 
		
		if (reception485Complete) {
			decode485Str();
			reception485Complete = false;
		}
		
		// Se varredura esta habilitada
		if (scan_status) {
			if (rs485_Timedout) {
				// Se fila de envio de comando esta vazia, faz pergunta
				if (rsRingBuffer_IsEmpty(rsBufferPtr)) {
					id_getNext(&IDs, &modulo);
				
					// Enviar requisicao de dados
					if (modulo->id != 0) {
						send485Cmd(modulo->id, modulo->tipo, '?', 0x00/*modulo_status_out[modulo->id]*/);
						
						rs485_Timedout = false;
					}
				}
				// Senao envia o proximo comando da fila
				else {
					// enviar dado do Buffer	
					rsRingBuffer_Remove(rsBufferPtr, data_out);
					RingBuffAddChar((RingBuff_t* const) tx485RingBuff, data_out, 10);
					USART_485_transmit();
					
					rs485_Timedout = false;
				}
			}
		}
    }
}

// PORTA SERIAL 232
void USART_232_init(unsigned int baud_rate) {
	// Definir Baud Rate
	UBRR0H = (unsigned char) (baud_rate >> 8);
	UBRR0L = (unsigned char) baud_rate;
	
	// Habilitar Receptor e Transmissor
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	
	// Definir tamanho do frame de dados (8 bits) e 1 Stop bit (condicao inicial)
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
	
	// Habilitar Interrupcoes do Receptor e do Transmissor
	UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);
}

void RingBuffAddChar(RingBuff_t* const buffer, uint8_t* str, uint8_t tam) {
	uint8_t i;
	
	if (RingBuffer_GetMaxLength() - RingBuffer_GetCount(buffer) > tam) {
		for (i = 0; i < tam; i++) {
			RingBuffer_Insert(buffer, str[i]);
		}
	}
}

void USART_232_transmit() {
	if (!RingBuffer_IsEmpty((RingBuff_t* const) tx232RingBuff)) {
		UCSR0B |= (1 << UDRIE0);
	}
}

ISR(USART0_UDRE_vect) {
	if (!RingBuffer_IsEmpty((RingBuff_t* const) tx232RingBuff)) {
		UDR0 = RingBuffer_Remove((RingBuff_t* const) tx232RingBuff);
	}
	else {
		UCSR0B &= ~(1 << UDRIE0);
	}
}

ISR(USART0_TX_vect) {
	// TODO:: ...
}

ISR(USART0_RX_vect) {
	char rxByte;
	
 	rxByte = UDR0;
	
	// Se string de recepcao esta vazia
	if (rx232StrPtr - rx232Str == 0) {
		// Se o primeiro char é 0x05 (char de inicio de msg)
		if (rxByte == 0xFF) {
			memcpy(rx232StrPtr, &rxByte, sizeof(char));
			rx232StrPtr++;
		}
		// Limpa string de recepcao e mostra msg erro 
		else {
			limpa_232Str();
			
			#ifdef DEBUG
				qde_blink = 16;
				VNET_backup = PORTC & (1 << HWD_NET);
				PWM_Start_Timeout();
			#endif
		}
	}
	// ...
	else if (rx232StrPtr - rx232Str == 1) {
		// Se o segundo char é 0x05 (char de inicio de msg)
		if (rxByte == 0x05) {
			memcpy(rx232StrPtr, &rxByte, sizeof(char));
			rx232StrPtr++;
		}
		// Limpa string de recepcao e mostra msg erro 
		else {
			limpa_232Str();
			
			#ifdef DEBUG
			qde_blink = 16;
			VNET_backup = PORTC & (1 << HWD_NET);
			PWM_Start_Timeout();
			#endif
		}
	}
	// Se string de rececpao está entre segundo e ultimo char vai armazenando na string de recepcao
	else if (rx232StrPtr - rx232Str > 0 && rx232StrPtr - rx232Str < 9) {
		memcpy(rx232StrPtr, &rxByte, sizeof(char));
		rx232StrPtr++;
		
		// Se string de recepcao atingiu tamanho max (= 10)
		if (rx232StrPtr - rx232Str == 10) {
			// Validação dos caracteres de inicio e fim de msg
			if (rx232Str[0] == 0xFF && rx232Str[1] == 0x05 && rx232Str[9] == 0x0D)	{
				reception232Complete = true;
			}
			else {
				// ToDo:: Solicitar reenvio da string
			}
		}
	}
	// Se string de recepcao for maior que gera msg erro
	else if (rx232StrPtr - rx232Str >= 10) {
		limpa_232Str();
		
		#ifdef DEBUG
			qde_blink = 20;
			VNET_backup = PORTC & (1 << HWD_NET);
			PWM_Start_Timeout();
		#endif
	}
}

// PORTA SERIAL 485
void USART_485_init(unsigned int baud_rate) {
	// Definir Baud Rate
	UBRR1H = (unsigned char) (baud_rate >> 8);
	UBRR1L = (unsigned char) baud_rate;
	
	// Habilitar Receptor e Transmissor
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	
	// Definir tamanho do frame de dados (8 bits) e 1 Stop bit (condicao inicial)
	UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);
	
	// Habilitar Interrupcoes do Receptor e do Transmissor
	UCSR1B |= (1 << RXCIE1) | (1 << TXCIE1);
}

void USART_485_transmit() {
	if (!RingBuffer_IsEmpty((RingBuff_t* const) tx485RingBuff)) {
		// Habilita Transmissao MAX485
		PORTC |= (1 << PC0);
		_delay_us(1);
		UCSR1B |= (1 << UDRIE1);
	}
}

ISR(USART1_TX_vect) {
	// Habilita Recepcao MAX485
	PORTC &= ~(1 << PC0);
	
	// Ao termino do envio de dados, comeca-se a conta o Timeout
	rs485_Start_Timeout();
}

ISR(USART1_UDRE_vect) {	
	uint8_t data;
	
	if (!RingBuffer_IsEmpty((RingBuff_t* const) tx485RingBuff)) {
		data = RingBuffer_Remove((RingBuff_t* const) tx485RingBuff);
		UDR1 = data;
		
		if (data == 0xFF) {
			last_sent_msg.count = 0;
			last_sent_msg.msg[last_sent_msg.count] = data;
			last_sent_msg.count++;
		}
	}
	else {
		UCSR1B &= ~(1 << UDRIE1);
	}
}

ISR(USART1_RX_vect) {
	char rxByte;
	
	rxByte = UDR1;
	
	// Se string de recepcao esta vazia
	if (rx485StrPtr - rx485Str == 0) {
		// Se o primeiro char é 0xFF (char de inicio de msg)
		if (rxByte == 0xFF) {
			memcpy(rx485StrPtr, &rxByte, sizeof(char));
			rx485StrPtr++;
		}
		// Limpa string de recepcao e mostra msg erro 11
		else {
			limpa_485Str();
			
			#ifdef DEBUG
				qde_blink = 4;
				VNET_backup = PORTC & (1 << HWD_NET);
				PWM_Start_Timeout();
			#endif
		}
	}
	// Se é o segundo char da string de recepcao
	else if (rx485StrPtr - rx485Str == 1) {
		// Se o segundo char é 0x06 (char feedback de comando)
		if (rxByte == 0x06) {
			memcpy(rx485StrPtr, &rxByte, sizeof(char));
			rx485StrPtr++;
		}
		// Limpa string de recepcao e mostra msg erro 10
		else {
			limpa_485Str();
			
			#ifdef DEBUG
				qde_blink = 8;
				VNET_backup = PORTC & (1 << HWD_NET);
				PWM_Start_Timeout();
			#endif
		}
	}
	// Se string de rececpao está entre segundo e ultimo char vai armazenando na string de recepcao
	else if (rx485StrPtr - rx485Str > 1 && rx485StrPtr - rx485Str < 10) {
		memcpy(rx485StrPtr, &rxByte, sizeof(char));
		rx485StrPtr++;
		
		// Se string de recepcao atingiu tamanho max (= 10)
		if (rx485StrPtr - rx485Str == 10) {
			// Validação dos caracteres de inicio e fim de msg
			if (rx485Str[0] == 0xFF && rx485Str[1] == 0x06 && rx485Str[9] == 0x0D)	{
				reception485Complete = true;
			}
			else {
				// ToDo:: Solicitar reenvio de String
			}
		}
	}
	// Se string de recepcao for maior que gera msg erro 0x08
	else if (rx485StrPtr - rx485Str >= 10) {
		limpa_485Str();
		
		#ifdef DEBUG
			qde_blink = 12;
			VNET_backup = PORTC & (1 << HWD_NET);
			PWM_Start_Timeout();
		#endif
	}
}

void Make485Cmd(rsRingBuffer_Data_t *data, char ID, char tipo, char CMD1, char CMD2) {
// 	char check;
// 	uint8_t i;
	
	data[0] = 0xFF;
	data[1] = 0x05;
	data[2] = ID;
	data[3] = tipo;
	data[4] = CMD1;
	data[5] = CMD2;
	data[6] = 0x00;
	data[7] = 0x00;
	
// 	check = data[0];
// 	for (i = 1; i <= 7; i++) {
// 		check ^= data[i];
// 	}
// 	
// 	data[8] = check;

	data[8] = VIRTUAL_CHECKSUM;
	data[9] = 0x0D;
}

void send485Cmd(char ID, char tipo, char CMD1, char CMD2) {
	uint8_t feedback[ELEMENT_SIZE];
	
	Make485Cmd(feedback, ID, tipo, CMD1, CMD2);
	
	RingBuffAddChar((RingBuff_t* const) tx485RingBuff, feedback, 10);
	USART_485_transmit();
}

//void Make232Cmd(rsRingBuffer_Data_t *data, char CMD1, char CMD2, char PARAM_1, char PARAM_2, char PARAM_3) {
void Make232Cmd(rsRingBuffer_Data_t *data, char CMD_ID, char CMD_TYPE, char CMD1, char CMD2, char CMD3, char CMD4) {
// 	char check;
// 	uint8_t i;
	
// 	data[0] = 0x06;
// 	data[1] = CMD1;
// 	data[2] = CMD2;
// 	data[3] = PARAM_1;
// 	data[4] = PARAM_2;
// 	data[5] = PARAM_3;
// 	
// 	check = data[0];
// 	for (i = 1; i < 6; i++) {
// 		check ^= data[i];
// 	}
// 	
// 	data[6] = check;
// 	data[7] = 0x0D;
	
	data[0] = 0xFF;
	data[1] = 0x06;
	data[2] = CMD_ID;
	data[3] = CMD_TYPE;
	data[4] = CMD1;
	data[5] = CMD2;
	data[6] = CMD3;
	data[7] = CMD4;
	
// 	check = data[0];
// 	for (i = 1; i <= 7; i++) {
// 		check ^= data[i];
// 	}
// 	
// 	data[8] = check;

	data[8] = VIRTUAL_CHECKSUM;
	data[9] = 0x0D;
	
	// Se msg nao for msg keep alive, reinicia contagem de tempo
	if (CMD1 != 0x13) {
		keep_alive_Stop_Timeout();
		keep_alive_Start_Timeout();
	}
}

void limpa_232Str() {
	uint8_t i;
	rx232StrPtr = rx232Str;
	for (i = 0; i < 10; i++) {
		rx232Str[i] = 0x00;
	}
}

void decode232Str() {
//	uint8_t i;
	char check;
	uint8_t result;
	
	uint8_t feedback[10];
	
	if (rx232Str[0] == 0x05) {
			
// 		check = rx232Str[0];
// 		for (i = 1; i < 8; i++) {
// 			check ^= rx232Str[i];
// 		}

		check = VIRTUAL_CHECKSUM;
		
		if (check == rx232Str[8]) {
			// ####################### TO BRIDGE #########################
			if (rx232Str[2] == TO_BRIDGE) {
				if (rx232Str[3] == 0x11) {
					switch (rx232Str[4]) {
						case BRIDGE_ADD:
							result = id_insert(&IDs, rx232Str[5], rx232Str[6]);
							eeprom_write_byte((uint8_t *) (uint16_t) rx232Str[5], (uint8_t) 1);
							break;
							
						case BRIDGE_REM:
							result = id_remove(&IDs, rx232Str[5], rx232Str[6]);
							eeprom_write_byte((uint8_t *) (uint16_t) rx232Str[5], (uint8_t) 0);
							break;
							
						default:
							result = NAK;
							break;
					}
					
					// ToDo:: armazenar lista de IDs e TIPOS na memoria EEPROM
					
					Make232Cmd(feedback, TO_BRIDGE, 0x11, result, rx232Str[5], rx232Str[6], 0x00);
				}
				else if (rx232Str[3] == 0x12) {
					// ToDo:: Verificar se já existe varredura habilitada, se não: enviar comando RS485 para habilitar rs485Timedout
					switch (rx232Str[4]) {
						case BRIDGE_ADD: {
							scan_status = true;
							rs485_Timedout = true;	// temporario, ver ToDo::
							PORTC = PINC | (1 << HWD_NET);
							break;
						}
						
						case BRIDGE_REM:
							scan_status = false;
							PORTC = PINC & ~(1 << HWD_NET);
							break;
						
						default:
							// Gerar mensagem de erro
							break;
					}
					
					Make232Cmd(feedback, TO_BRIDGE, 0x12, scan_status ? ACK : NAK, rx232Str[5], rx232Str[6], 0x00);
				}
				// Keep Alive true
				else if (rx232Str[3] == 0x13) {
					keep_alive_Stop_Timeout();
					
					// Reativa varredura
					scan_status = true;
					PORTC = PINC | (1 << HWD_NET);
					
					keep_alive_Start_Timeout();
				}
				else {
					/* Não é um tipo de comando valido para o Bridge */
					Make232Cmd(feedback, TO_BRIDGE, ERR, ERR_CMD2, rx232Str[5], rx232Str[6], 0x00);
				}
				
				RingBuffAddChar((RingBuff_t* const) tx232RingBuff, feedback, 10);
				USART_232_transmit();
			}
			// ######################### TO GO ###########################
			else if (rx232Str[2] == TO_GO) {
				// TODO::
				// Tipos de módulo, como só tem IORL8 com tipo = 1 por enquanto, temoss < 2
				// Se modulo é rele
				if (rx232Str[3] > 0 && rx232Str[3] < 2) {
					Make485Cmd(data_in, rx232Str[4], rx232Str[3], rx232Str[5], rx232Str[6]);
					if (!rsRingBuffer_IsFull(rsBufferPtr)) {
						rsRingBuffer_Insert(rsBufferPtr, data_in);
						
						Make232Cmd(feedback, TO_BRIDGE,  rx232Str[3],  rx232Str[4], rx232Str[5], rx232Str[6], 0x00);
					}
					else {
						//ToDo:: Gerar um log de erro: Buffer cheio
						Make232Cmd(feedback, TO_BRIDGE, ERR, 0xE0, rx232Str[5], rx232Str[6], 0x00);
					}
				}
				else {
					// Tipo Inválido
					Make232Cmd(feedback, TO_BRIDGE, ERR, ERR_CMD2, rx232Str[5], rx232Str[6], 0x00);					
				}
				
				RingBuffAddChar((RingBuff_t* const) tx232RingBuff, feedback, 10);
				USART_232_transmit();
			}
			else {
				// Não é um tipo de comando válido a ser enviado na rede RS485
				Make232Cmd(feedback, TO_BRIDGE, ERR, ERR_CMD1, rx232Str[5], rx232Str[6], 0x00);
				RingBuffAddChar((RingBuff_t* const) tx232RingBuff, feedback, 10);
				USART_232_transmit();
			}
		}
		else {
			// Erro no Check Sum
			Make232Cmd(feedback, TO_BRIDGE, ERR, ERR_CHECKSUM, rx232Str[5], rx232Str[6], 0x00);
			RingBuffAddChar((RingBuff_t* const) tx232RingBuff, feedback, 10);
			USART_232_transmit();
		}
	}
	else {
		/* Não é uma requisição */
	}
	
	limpa_232Str();
}

void limpa_485Str() {
	uint8_t i;
	rx485StrPtr = rx485Str;
	for (i = 0; i < 10; i++) {
		rx485Str[i] = 0x00;
	}
}

void decode485Str() {
//	uint8_t i;
	uint8_t check;
	uint8_t temp;
	bool send_232;
	
// 	check = rx485Str[0];
// 	for (i = 1; i <= 7; i++) {
// 		check ^= rx485Str[i];
// 	}

	check = VIRTUAL_CHECKSUM;
	
	send_232 = false;
	if (check == rx485Str[8]) {
		if (rx485Str[1] == 0x06) {
			//	Feedback entradas
			// Juntar byte: Upper and Lower significant bits
			temp = rx485Str[5];
			temp = temp << 4;
			temp = temp | (0x0F & rx485Str[4]);
		
			// Comparar com o status armazenado
			if (modulo_status_in[rx485Str[2]] != temp) {
				modulo_status_in[rx485Str[2]] = temp;
				send_232 = true;
			}
		
			//	Feedback saidas
			// Juntar byte: Upper and Lower significant bits
			temp = rx485Str[7];
			temp = temp << 4;
			temp = temp | (0x0F & rx485Str[6]);
		
			// Comparar com o status armazenado
			if (modulo_status_out[rx485Str[2]] != temp) {
				modulo_status_out[rx485Str[2]] = temp;
				send_232 = true;
			}
		
			// Ativar LED de status
			PORTC |= (1 << HWD_NET);
		
			// Ativar quando receber string valida da porta serial
			// ?? O comando abaixo esta certo ??
			rs485_Stop_Timeout();
			rs485_Timedout = true;
		}
		else if (rx485Str[1] == NAK) {
			// Reenviar string
		}
	}
	
	if (send_232) {
		RingBuffAddChar((RingBuff_t* const) tx232RingBuff, rx485Str, 10);
		USART_232_transmit();
	}
	
	// Inicializar dados recebidos
	limpa_485Str();
}


void REG16_WriteOCR1A(unsigned int i) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		OCR1A = i;
	}
}

void rs485_Start_Timeout() {
	TCCR1B |= (1 << CS10) | (1 << CS12);
}

void rs485_Stop_Timeout() {
	TCCR1B &= ~( (1 << CS10) | (1 << CS12) );
	
	// Zerar Timer
	TCNT1 = 0;
}

ISR(TIMER1_COMPA_vect) {
	PORTC = PINC & ~(1 << HWD_NET);
	
	rs485_Timedout = true;
	rs485_Stop_Timeout();
}

void REG16_WriteOCR3C(unsigned int i) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		OCR3C = i;
	}
}

void PWM_Start_Timeout() {
	TCCR3B |= (1 << CS30) | (1 << CS32);
}

void PWM_Stop_Timeout() {
	TCCR3B &= ~( (1 << CS30) | (1 << CS32) );
	
	// Zerar Timer
	TCNT3 = 0;
}

ISR(TIMER3_COMPC_vect) {
//	int i;
// 	id_item *modulo;
// 	
// 	PORTC = PINC ^ (1 << HWD_NET);
// 	
// 	if (kk ==  0) {
// 		for ( i = 0; i < 9; i++ ) {
// 			id_getNext(&IDs, &modulo);
// 			MakeStrCmd(modulo->id, modulo->tipo);
// 		}
// 	}
// 	
// 	else if (kk ==  1) {
// 		MakeStrCmd(0xAA, id_insert(&IDs, 0x01, 0x01));
// 		for ( i = 0; i < 9; i++ ) {
// 			id_getNext(&IDs, &modulo);
// 			MakeStrCmd(modulo->id, modulo->tipo);
// 		}
// 	}
// 	
// 	else if (kk ==  2) {
// 	MakeStrCmd(0xAA, id_insert(&IDs, 0x02, 0x01));
// 		for ( i = 0; i < 9; i++ ) {
// 			id_getNext(&IDs, &modulo);
// 			MakeStrCmd(modulo->id, modulo->tipo);
// 		}
// 	}
// 	
// 	else if (kk ==  3) {
// 		id_remove(&IDs, 0x01, 0x01);
// 		for ( i = 0; i < 9; i++ ) {
// 			id_getNext(&IDs, &modulo);
// 			MakeStrCmd(modulo->id, modulo->tipo);
// 		}
// 	}
// 	else {
// 		PWM_Stop_Timeout();
// 	}
// 	
// 	kk++;
// 	
// 	TCNT3 = 0;
	
	#ifdef DEBUG
		PORTC = PINC ^ (1 << HWD_NET);
		qde_blink--;
		
		TCNT3 = 0;
	
		if(qde_blink == 0) {
			PWM_Stop_Timeout();
			PORTC = (1 << HWD_NET) & (1 << VNET_backup) ? PINC | (1 << HWD_NET) : PINC & ~(1 << HWD_NET);
		}
	#else
		PWM_Stop_Timeout();	
	#endif
}

void MakeStrCmd(char ID, char tipo) {
	uint8_t data[7];
	data[0] = 'I';
	data[1] = ':';
	data[2] = ID;
	data[3] = ' ';
	data[4] = 'T';
	
	data[5] = tipo;
	data[6] = ' ';
	
	RingBuffAddChar((RingBuff_t* const) tx485RingBuff, data, 7);
	USART_485_transmit();
}

// ISR(TIMER3_COMPC_vect) {
// 	// Enviar requisicao de dados
// 	if (modulo->id != 0) {
// 		send485Cmd(modulo->id, modulo->tipo, '?', modulo_status_out[modulo->id]);
// 	}
// 	
// 	//
// 	Make485Cmd(data_in, 0x01, 0x01, rele_cmd, rele_out[kk]);
// 	if (!rsRingBuffer_IsFull(rsBufferPtr)) {
// 		rsRingBuffer_Insert(rsBufferPtr, data_in);
// 	}
// 	if (!rsRingBuffer_IsEmpty(rsBufferPtr)) {
// 		rsRingBuffer_Remove(rsBufferPtr, data_out);
// 		RingBuffAddChar((RingBuff_t* const) tx485RingBuff, data_out, 7);
// 		USART_485_transmit();
// 	}
// 	
// 	
// 	kk++;
// 	if (kk > 7) {
// 		kk = 0;
// 		
// 		if (ll == 1) {
// 			rele_cmd = 0xFF;
// 			ll = 0;
// 		}
// 		else {
// 			rele_cmd = 0xFE;
// 			ll = 1;
// 		}
// 	}
// 	
// 	TCNT3 = 0;
// }

void keep_alive_Start_Timeout() {
	TCCR0 |= (1 << CS00) | (1 << CS01) | (1 << CS02);
}

void keep_alive_Stop_Timeout() {
	TCCR0 &= ~( (1 << CS00) | (1 << CS01) | (1 << CS02) );
	
	// Zerar Timer
	TCNT0 = 0;
	
	keep_alive = 0;
}

ISR(TIMER0_COMP_vect) {
	uint8_t msg[8];
	keep_alive++;
	TCNT0 = 0;
	
	// 500cs = 5s sem receber msg externa envia msg keep alive
	if (keep_alive == 500) {
		// Timeout - Enviar msg keep alive
		Make232Cmd(msg, TO_BRIDGE, 0x13, 0x0F, 0x00, 0x00, 0x00);
		RingBuffAddChar((RingBuff_t* const) tx232RingBuff, msg, 8);
		USART_232_transmit();
	}
	// Espera ate 530cs, +300ms = +30cs pela resposta, se nao receber desativa varredura
	else if (keep_alive > 529) {
		scan_status = false;
		PORTC = PINC & ~(1 << HWD_NET);
		
		// Existe necessidade de transmitir? Nao houve resposta anterior...
// 		Make232Cmd(feedback, TO_BRIDGE, 0x12, scan_status ? ACK : NAK, 0x00, 0x00);
// 		RingBuffAddChar((RingBuff_t* const) tx232RingBuff, feedback, 8);
// 		USART_232_transmit();
		
		keep_alive = 0;
	}
}
