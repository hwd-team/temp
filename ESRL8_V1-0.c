/*
 * ESRL8_R1.c
 *
 * Created: 08/05/2015 17:27:24
 *  Author: Webert Brito e Diego Castro
 */ 



#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include <avr/interrupt.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include "LightweightRingBuff.h"

#define BAUD 115200
//#define BAUD 921600
#define myUBRR F_CPU/16/BAUD-1

#define PWM_MAX	0x03FF

#define VNET_LED		PB5

#define C_ON	0xF0
#define C_OFF	0x0F
#define FB_MASK	0xE0

#define VIRTUAL_CHECKSUM 0xAA

#define DEBUG	1

//--------------------------------

uint8_t	t_count;

unsigned char mem_local_input;	// ARMAZENA ULTIMO STATUS DAS ENTRADAS LOCAIS
unsigned char mem_ext_input;	// ARMAZENA ULTIMO STATUS DAS ENTRADAS EXTERNAS
unsigned char mem_bot_setup;	// ARMAZENA ULTIMO STATUS DO BOTAO SETUP
unsigned char setup;			// 1 = SETUP ON; 0 = SETUP OFF;
unsigned char ID;				// ARMAZENA ID ATUAL;
unsigned char ID_mem;			// ULTIMO ID VÁLIDO
unsigned char mem_bot_digH;		// DIGITO MAIS SIGNIFICATIVO DO ID
unsigned char mem_bot_digL;		// DIGITO MENOS SIGNIFICATIPO DO ID
unsigned int blink;				// CONTADOR DE 0000 A FFFF QUE FAZ O DISPLAY PISCAR

uint8_t eeprom_data;

bool receptionComplete;
volatile RingBuff_t *txRingBuff, txVRingBuff; // RING BUFFER QUE ARMAZENA STRINGS A SEREM ENVIADAS
//volatile RingBuff_t *rxRingBuff, rxVRingBuff; // RING BUFFER QUE ARMAZENA STRINGS RECEBIDAS

char str[12];
char *rxStr;
char *rxStrPtr;
uint8_t tipo;

bool VNET_Status;

// PWM
uint16_t pwm_fade_time;			// unidade cs (centisegundo)
uint16_t pwm_temp_fade_time;	// unidade cs (centisegundo)
uint16_t pwm_final_fade_time;	// unidade cs (centisegundo)

// PROTOTIPO DAS FUNÇÕES

void checkLocalInput();
void checkExtInputInterruptor();
void checkExtInputPulsador();
void checkSetup();
void blinkDisplay();
void desligaDisplay();
void escreveID();
void alteraID_H();
void alteraID_L();
void timeout_go();
void reset_timeout();
void limpa_str();

// FUNÇÕES PORTA SERIAL
void USART_init(unsigned int baud_rate);
void USART_transmit();
//void RingBuffAddString(RingBuff_t* const buffer, char* str);
void RingBuffAddChar(RingBuff_t* const buffer, char* str, uint8_t strSize);
void decodeInputStr();
void sendFB();

void resend_rqst();

void mystrcat(char* str, char ch);

void input_Start_Timeout();
void input_Stop_Timeout();

void REG16_WriteOCR1A(unsigned int i);
void rs485_Start_Timeout();
void rs485_Stop_Timeout();

void REG16_WriteOCR3C(unsigned int i);
void PWM_Start_Timeout();
void PWM_Stop_Timeout();


//================================

// PROGRAMA

int main(void)
{
	// Definir Tipo do módulo
	tipo = 1;
	
	// 
	VNET_Status = false;
	
	// Timer0
	// Configura Timer1 para o modo CTC
	TCCR0 |= (1 << WGM01);
	// Habilita interrupcao CTC para o comparador A
	TIMSK |= (1 << OCIE0);
	// Define o limite de contagem do CTC. Configuracao de 10ms a um clock de 14,7456 MHz e prescaler de 1024
	OCR0 = 0x8F;
	input_Stop_Timeout();
	
	// Timer1
	// Configura Timer1 para o modo CTC
	TCCR1B |= (1 << WGM12);
	// Habilita interrupcao CTC para o comparador A
	TIMSK |= (1 << OCIE1A);
	// Define o limite de contagem do CTC. Configuracao de 500ms a um clock de 14,7456 MHz e prescaler de 1024
	REG16_WriteOCR1A(7199);
	rs485_Stop_Timeout();
	
	// Timer2
	// Configura Timer2 para o modo CTC
	TCCR2 |= (1 << WGM21);
	// Habilita interrupcao CTC para o comparador A
	TIMSK |= (1 << OCIE2);
	// Define o limite de contagem do CTC. Configuracao de 10ms a um clock de 14,7456 MHz e prescaler de 1024
	OCR0 = 0x8F;
	input_Stop_Timeout();
	
		// Timer3
		// Configura Timer3 para o modo CTC
		TCCR3B |= (1 << WGM32);
		// Habilita interrupcao CTC para o comparador C
		ETIMSK |= (1 << OCIE3C);
		// Define o limite de contagem do CTC. Configuracao de 1s a um clock de 14,7456 MHz e prescaler de 1024
		REG16_WriteOCR3C(14399);
		PWM_Stop_Timeout();
	
// 	// Timer3
// 	// Configurado a um clock de 14,7456 MHz e prescaler 8 (PWM_Start_Timeout) gerando uma onda de 1,8 KHz 
// 	// Configura o Timer3 para o modo FastPWM com limite de contagem em 0x03FF, 10 bits de resolução (WGM33:0 = 7)
// 	TCCR3A |= (1 << WGM31) | (1 << WGM30);
// 	TCCR3B |= (1 << WGM32);
// 	
// 	// Configura saida OC3C (PE5) para o modo non-inverting (COM3C1:0 = 2)
// 	TCCR3A |= (1 << COM3C1);
// 	
// 	//OCR3C é a % do PWM, alterar valor na interrupção TOV
// 	pwm_fade_time = PWM_MAX;
// 	REG16_WriteOCR3C(pwm_fade_time);
// 	PWM_Stop_Timeout();
// 	
// 	// Habilita interrupcao overflow, TOV3, para o Timer3
// 	ETIMSK |= (1 << TOIE3);
	
	
	// Iniciar String de recepção da porta serial
	rxStr = str;		// Inicialização do ponteiro
	rxStrPtr = str;
	receptionComplete = false;
	
	cli();
	
	SFIOR	= SFIOR & ~(1 << PUD); // Garantir que os pull-ups serão habilitados
	ADCSRA	= 0x00;		// Desabilitar Conversor A/D
	
	// Habilitar saida PWM, RS485_EN e RS485_END como saida
	DDRE |= (1 << PE5) | (1 << PE6) | (1 << PE7);
	// Setar MAX485 como recepcao
	PORTE &= ~(1 << PE6);
	PORTE |= (1 << PE5); //HABILITA PWM (PWM_Start_Timeout())
	
	// Pull-up Entradas
	PORTF = 0xFF; // ENTRADA EXT
	PORTA = 0xFF; // ENTRADA LOCAL
	PORTB |= (1 << PB4); // BOTAO SETUP
	
	// LED HWD NET E BOTAO SETUP
	DDRB |= (1 << PB5);
	
	// Entradas locais (push buttons)
	DDRA = 0x00;
	
	// Entradas externas
	DDRF = 0x00;
	
	// Saidas rele
	DDRC = 0xFF;
	
	// Displays 7 segmentos
	DDRD = 0x00;
	DDRD = (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
	
	
	PORTD |= (1 << PD6) | (1 << PD7); // BOTAO CONFIG ID
	
	PORTD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3); // ENVIA 'VAZIO'PARA OS DISPLAYS
	PORTD |= (1 << PD4) | (1 << PD5); // HABILITA BUFFER DOS DISPLAYS
	_delay_ms(1);
	PORTD &= ~(1 << PD4 | 1 << PD5);


	mem_ext_input = PINF; // ARMAZENA LEITURA INICIAL DOS INTERRUPTORES NA MEMORIA
	mem_bot_setup = 0x00;
	mem_bot_digL = 0x00;
	mem_bot_digH = 0x00;
	setup = 0x00;
	blink = 0x0000;
	
	eeprom_data = eeprom_read_byte((uint8_t*)00); // BUSCA VALOR DA ID NA EEPROM
	
	if (eeprom_data == 0xFF) {
		ID = 0x01;
		eeprom_write_byte ((uint8_t*) 00, ID); // ARMAZENA NOVO ID NA EEPROM
	}
	else {
		ID = eeprom_data;
	} 
	ID_mem = ID;
	escreveID();
	
	// PORTA SERIAL
	txRingBuff = &txVRingBuff;
	RingBuffer_InitBuffer((RingBuff_t* const) txRingBuff);
	
// 	rxRingBuff = &rxVRingBuff;
// 	RingBuffer_InitBuffer((RingBuff_t* const) rxRingBuff);
	
	USART_init(myUBRR);
	
	sei();
	
	//sendFB();
	
	while(1)
	{
		checkLocalInput();
		//checkExtInputInterruptor();
		checkExtInputPulsador();
		checkSetup();
		blinkDisplay();
		
		if (receptionComplete) {
			decodeInputStr();
			receptionComplete = false;
		}
	}
}

// FUNÇÕES

void checkLocalInput() {
	unsigned char readed_input;
	unsigned char i;
	unsigned char teste;
	
	readed_input = PINA;
	if (readed_input != mem_local_input) {
		for (i = 0; i < 8; i++) {
			// Se porta de entrada está diferente da ultima leitura
			// As entradas possuem pull-up, por isto tem-se de inverter a variavel
			// Analise dos pinos de entrada (individualemnte)
			// Borda de subida
			if ( (~readed_input & (1 << i)) & (mem_local_input & (1 << i)) ) {
				input_Start_Timeout();
				
				// Atuaaliza o ultimo estado das entradas
				mem_local_input = readed_input;				
			}
			// Borda de descida
			else if ( (readed_input & (1 << i)) & (~mem_local_input & (1 << i)) ) {
				if (t_count >= 4) {
					teste = PINC;
					PORTC = teste ^ (1 << i);
				}
				
				input_Stop_Timeout();
				
				// Atuaaliza o ultimo estado das entradas
				mem_local_input = readed_input;
			}
		}
	}
}

void checkExtInputPulsador() {
	unsigned char readed_input;
	unsigned char i;
	unsigned char teste;
	
	readed_input = PINF;
	if (readed_input != mem_ext_input) {
		for (i = 0; i < 8; i++) {
			// Se porta de entrada está diferente da ultima leitura
			// As entradas possuem pull-up, por isto tem-se de inverter a variavel
			// Analise dos pinos de entrada (individualemnte)
			// Borda de subida
			if ( (~readed_input & (1 << i)) & (mem_ext_input & (1 << i)) ) {
				input_Start_Timeout();
				
				// Atuaaliza o ultimo estado das entradas
				mem_ext_input = readed_input;
			}
			// Borda de descida
			else if ( (readed_input & (1 << i)) & (~mem_ext_input & (1 << i)) ) {
				if (t_count >= 4) {
					if (!VNET_Status) {
						teste = PINC;
						PORTC = teste ^ (1 << i);
					}
				}
				
				input_Stop_Timeout();
				
				// Atuaaliza o ultimo estado das entradas
				mem_ext_input = readed_input;
			}
		}
	}
}

void checkExtInputInterruptor() {
	unsigned char readed_input;
	unsigned char i;
	unsigned char teste;
	
	readed_input = PINF;
	if (readed_input != mem_ext_input) {
		for (i = 0; i < 8; i++) {
			// Se porta de entrada está diferente da ultima leitura
			// As entradas possuem pull-up, por isto tem-se de inverter a variavel
			// Analise dos pinos de entrada (individualemnte)
			// Borda de subida
			if ( (~readed_input & (1 << i)) & (mem_ext_input & (1<<i)) ) {
				_delay_ms(40);
				readed_input = PINF;
				if (~readed_input & (1 << i)) {
					teste = PINC;
					PORTC = teste ^ (1 << i);
				}
			}
			// Borda de descida
			else if ( (readed_input & (1 << i)) & (~mem_ext_input & (1<<i)) ) {
				_delay_ms(40);
				readed_input = PINF;
				if ( readed_input & (1 << i)) {
					teste = PINC;
					PORTC = teste ^ (1 << i);
				}
			}
		}
	}
	// Atuaaliza o ultimo estado das entradas
	mem_ext_input = readed_input;
}

void checkSetup()	{
	unsigned char readed_input;
	
	readed_input = PINB;
	
	if ((readed_input & (1 << 4)) != mem_bot_setup) {
		if (~readed_input & (1 << 4)) {
			_delay_ms(40);
			readed_input = PINB;
			if (~readed_input & (1 << 4)) {
				setup = ~setup;					// INVERTE O STATUS;
			}
			else if ( readed_input & (1 << 4)){

			}
		}
	}
	mem_bot_setup = (readed_input & (1 << 4));
}

void blinkDisplay() {
	if(setup & 1) {
		alteraID_H();	// VERIFICA ALTEREÇÃO DE ID PELO BOTÃO MAIS SIGNIFICATIVO
		alteraID_L();
		blink++;
		if (blink == 0x7FFF)
		{
			escreveID();
		}
		else if (blink == 0xFFFF)
		{
			desligaDisplay();
			blink = 0x0000;
		}
	}
	else {
		//escreveID();
		//desligaDisplay();
		if (ID != ID_mem) {
			eeprom_write_byte ((uint8_t*) 00, ID); // ARMAZENA NOVO ID NA EEPROM
			ID_mem = ID;
		}
	}
}

void desligaDisplay() {
	
	PORTD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3); // ENVIA 'VAZIO'PARA OS DISPLAYS
	PORTD |= (1 << PD4) | (1 << PD5); // HABILITA BUFFER DOS DISPLAYS
	_delay_ms(1);
	PORTD &= ~(1 << PD4 | 1 << PD5); // DESABILITA BUFFER DOS DISPLAYS
}

void escreveID() {
	unsigned char id_l;
	unsigned char id_h;
		
	id_l = ID & 0x0F; // SEPARA A PARTE MENOS SIGNIFICATIVA DO ID;
	
	PORTD = PORTD & 0xF0; // LIMPA A PARTE MENOS SIGNIFICATIVA DA PORTA
	PORTD = PORTD | id_l; // ESCREVE O ID MENOS SIGNIFICATIVO NA PORTA SEM ALTERAR A PARTE MAIS SIGNIFICATIVA
	
	PORTD |= (1 << PD5); // HABILITA DISPLAY MENOS SIGNIFICATIVO;
	_delay_ms(1);
	PORTD &= ~(1 << PD5); // DESABILITA DISPLAY MENOS SIGNIFICATIVO
	_delay_ms(1);
	
	
	id_h = ID & 0xF0; // SEPARA A PARTE MAIS SIGNIFICATIVA DO ID;
	id_h = (id_h >> 4); // DESCOLCA 4 BITS PRA DIREITA
	
	PORTD = PORTD & 0xF0; // LIMPA A PARTE MENOS SIGNIFICATIVA DA PORTA
	PORTD = PORTD | id_h; // ESCREVE O ID MAIS SIGNIFICATIVO NA PORTA SEM ALTERAR A PARTE MAIS SIGNIFICATIVA
	
	_delay_ms(1);
	PORTD |= (1 << PD4); // HABILITA DISPLAY MENOS SIGNIFICATIVO;
	_delay_ms(1);
	PORTD &= ~(1 << PD4); // DESABILITA DISPLAY MENOS SIGNIFICATIVO
	_delay_ms(1);
	
}

void alteraID_L() {
	unsigned char readed_input;
	unsigned char id_l;
	
	readed_input = PIND;
	
	if ((readed_input & (1 << 7)) != mem_bot_digL) {
		if (~readed_input & (1 << 7)) {
			_delay_ms(40);
			readed_input = PIND;
			if (~readed_input & (1 << 7)) {
				id_l = ID & 0x0F;
				id_l++;
				if (id_l == 0x0A)
				{
					id_l = 0x00;
				}
				ID = ID & 0xF0;
				ID = ID | id_l;
				
			}
			else if ( readed_input & (1 << 7)){

			}
		}
	}
	mem_bot_digL = (readed_input & (1 << 7));
}

void alteraID_H() {
	unsigned char readed_input;
	unsigned char id_h;
	
	readed_input = PIND;
	
	if ((readed_input & (1 << 6)) != mem_bot_digH) {
		if (~readed_input & (1 << 6)) {
			_delay_ms(40);
			readed_input = PIND;
			if (~readed_input & (1 << 6)) {
				id_h = ID & 0xF0;
				id_h = (id_h >> 4);
				id_h++;
				if (id_h == 0x0A)
				{
					id_h = 0x00;
				}
				ID = ID & 0x0F;
				id_h = (id_h << 4);
				ID = ID | id_h;
			}
			else if ( readed_input & (1 << 6)){

			}
		}
	}
	mem_bot_digH = (readed_input & (1 << 6));
}

// PORTA SERIAL
void USART_init(unsigned int baud_rate) {
	// Definir Baud Rate
	UBRR0H = (unsigned char) (baud_rate >> 8);
	UBRR0L = (unsigned char) baud_rate;
	
	// Habilitar Receptor e Transmissor
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	
	// Definir tamanho do frame de dados (8 bits) e 1 Stop bit (condicao inicial)
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
	
	// Habilitar Interrupcoes do Receptor e do Transmissor
	//UCSR0B |= (1 << RXCIE0);
	UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);
}

void USART_transmit() {
	if (!RingBuffer_IsEmpty((RingBuff_t* const) txRingBuff)) {
		// Habilita Transmissao MAX485
		PORTE |= (1 << PE6);
		_delay_us(1);
		UCSR0B |= (1 << UDRIE0);
	}
}

ISR(USART0_UDRE_vect) {	
	if (!RingBuffer_IsEmpty((RingBuff_t* const) txRingBuff)) {
		UDR0 = RingBuffer_Remove((RingBuff_t* const) txRingBuff);
	}
	else {
		UCSR0B &= ~(1 << UDRIE0);
	}
}

ISR(USART0_TX_vect) {
	// Habilita Recepcao MAX485
	PORTE &= ~(1 << PE6);
}

ISR(USART0_RX_vect) {
	char rxByte;
	
 	rxByte = UDR0;
	
	// Se string de recepcao esta vazia
	if (rxStrPtr - rxStr == 0) {
		// Se o primeiro char é 0xFF (char de inicio de msg)
		if (rxByte == 0xFF) {
			memcpy(rxStrPtr, &rxByte, sizeof(char));
			rxStrPtr++;
		}
		// Limpa string de recepcao e mostra msg erro 11
		else {
			limpa_str();
			
			#ifdef DEBUG1
				ID = 0x11;
				escreveID();
				ID = ID_mem;
			
				PWM_Start_Timeout();
			#endif
		}
	}
	// Se é o segundo char da string de recepcao 
	else if (rxStrPtr - rxStr == 1) {
		// Se o segundo char é 0x05 (char envio de comando)
		if (rxByte == 0x05) {
			memcpy(rxStrPtr, &rxByte, sizeof(char));
			rxStrPtr++;
		}
		// Limpa string de recepcao e mostra msg erro 10
		else {
			limpa_str();
			
			#ifdef DEBUG1
				ID = 0x10;
				escreveID();
				ID = ID_mem;
			
				PWM_Start_Timeout();
			#endif
		}
	}
	// Se string de rececpao está entre segundo e ultimo char vai armazenando na string de recepcao
	else if (rxStrPtr - rxStr > 1 && rxStrPtr - rxStr < 10) {
		memcpy(rxStrPtr, &rxByte, sizeof(char));
		rxStrPtr++;
		
		// Se string de recepcao atingiu tamanho max (= 10)
		if (rxStrPtr - rxStr == 10) {
			// Validação dos caracteres de inicio e fim de msg
			if (rxStr[0] == 0xFF && rxStr[1] == 0x05 && rxStr[9] == 0x0D)	{
				receptionComplete = true;
			}
			else {
				limpa_str();
				
				// Solicitar reenvio
				resend_rqst();
				
				#ifdef DEBUG
					// String Invalida
					ID = 0x20;
					escreveID();
					ID = ID_mem;
				
					PWM_Start_Timeout();
				#endif
			}
		}
	}
	// Se string de recepcao for maior que 10 gera msg erro 0x08
	else if (rxStrPtr - rxStr >= 10) {
		limpa_str();
		
		#ifdef DEBUG
			ID = 0x08;
			escreveID();
			ID = ID_mem;
			
			PWM_Start_Timeout();
		#endif
	}
}

void RingBuffAddChar(RingBuff_t* const buffer, char* str, uint8_t strSize) {
	uint8_t i;
	
	if (RingBuffer_GetMaxLength() - RingBuffer_GetCount(buffer) > strSize) {
		for (i = 0; i < strSize; i++) {
			RingBuffer_Insert(buffer, str[i]);
		}
	}
}

void decodeInputStr() {
//	uint8_t i;
	unsigned char saida, check;
	
	check = VIRTUAL_CHECKSUM;
// 	check = rxStr[0];
// 	for (i = 1; i <= 7; i++) {
// 		check ^= rxStr[i];
// 	}
	
	if (check == rxStr[8]) {
	
		if (rxStr[1] == 0x05) {
			if (rxStr[2] == ID) {
				if (rxStr[3] == tipo) {
					if (rxStr[4] == '?') {
				
						//PORTC = rxStr[5];
							
						sendFB();
						
						// Reseta TimeOut
						reset_timeout();
					}
					else if (rxStr[4] == C_ON) {
						saida = PINC;
						PORTC = saida | rxStr[5];
						
						sendFB();
						
						// Reseta TimeOut
						reset_timeout();
					}
					else if (rxStr[4] == C_OFF) {
						saida = PINC;
						PORTC = saida & (~rxStr[5]);
						
						sendFB();
						
						// Reseta TimeOut
						reset_timeout();
					}
					else {
						/* Não é um comando válido */
						#ifdef DEBUG
							ID = 0x02;
							escreveID();
							ID = ID_mem;
							
							PWM_Start_Timeout();
						#endif
					}
				}
				else {
					/* Não é o tipo correto: rele */
					#ifdef DEBUG
						ID = 0x03;
						escreveID();
						ID = ID_mem;
						
						PWM_Start_Timeout();
					#endif
				}
			}
			else {
				/* Não é o ID correto */
// 				#ifdef DEBUG
// 					ID = 0x04;
// 					escreveID();
// 					ID = ID_mem;
// 					PWM_Start_Timeout();
			}
		}
		else {
			/* Não é uma requisição */
//			#ifdef DEBUG
// 				ID = 0x05;
// 				escreveID();
// 				ID = ID_mem;
// 				
// 				PWM_Start_Timeout();
		}
	}
	else {
		/* CheckSum Inválido */
		#ifdef DEBUG
			ID = 0x06;
			escreveID();
			ID = ID_mem;
			
			PWM_Start_Timeout();
		#endif
	}
	
	limpa_str();
}

void limpa_str() {
	uint8_t i;
	rxStrPtr = str;
	for (i = 0; i < 10; i++) {
		rxStr[i] = 0x00;
	}
}

void sendFB() {
	char feedback[10];
// 	char check;
// 	uint8_t i;
	
	feedback[0] = 0xFF;
	feedback[1] = 0x06;
	feedback[2] = ID;
	feedback[3] = tipo;
	feedback[4] = FB_MASK | (0x0F &  (~ (char) mem_ext_input));
	feedback[5] = FB_MASK | (0x0F & ((~ (char) mem_ext_input) >> 4));
	feedback[6] = FB_MASK | (0x0F &  PINC);
	feedback[7] = FB_MASK | (0x0F & (PINC >> 4));
	
// 	check = feedback[0];
// 	for (i = 1; i <= 7; i++) {
// 		check ^= feedback[i];
// 	}
// 	
// 	feedback[8] = check;
	feedback[8] = VIRTUAL_CHECKSUM;
	feedback[9] = 0x0D;
	
	RingBuffAddChar((RingBuff_t* const) txRingBuff, feedback, 10);
	USART_transmit();
}

void resend_rqst() {
	char rqst[10];
// 	char check;
// 	uint8_t i;
	
	rqst[0] = 0xFF;
	rqst[1] = 0x15;		// NAK
	rqst[2] = ID;
	rqst[3] = tipo;
	rqst[4] = FB_MASK | (0x0F &  (~ (char) mem_ext_input));
	rqst[5] = FB_MASK | (0x0F & ((~ (char) mem_ext_input) >> 4));
	rqst[6] = FB_MASK | (0x0F &  PINC);
	rqst[7] = FB_MASK | (0x0F & (PINC >> 4));
	
// 	check = rqst[0];
// 	for (i = 1; i <= 7; i++) {
// 		check ^= rqst[i];
// 	}
// 	
// 	rqst[8] = check;
	rqst[8] = VIRTUAL_CHECKSUM;
	rqst[9] = 0x0D;
	
	RingBuffAddChar((RingBuff_t* const) txRingBuff, rqst, 10);
	USART_transmit();
}

void reset_timeout() {
	// Reiniciar Timer
	rs485_Stop_Timeout();
	rs485_Start_Timeout();
		
	// Atualizar status
	VNET_Status = true;
	PORTB |= (1 << VNET_LED); // LIGA LED NET
}


void REG16_WriteOCR1A(unsigned int i) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		OCR1A = i;
	}
}

void REG16_WriteOCR3C(unsigned int i) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		OCR3C = i;
	}
}

void input_Start_Timeout() {
	TCCR0 |= (1 << CS00) | (1 << CS01) | (1 << CS02);
}

void input_Stop_Timeout() {
	TCCR0 &= ~( (1 << CS00) | (1 << CS01) | (1 << CS02) );
	
	// Zerar Timer
	TCNT0 = 0;
	
	t_count = 0;
}

ISR(TIMER0_COMP_vect) {
	t_count++;
	TCNT0 = 0;
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
	PORTB = PINB & ~(1 << VNET_LED);
	
	VNET_Status = false;
	rs485_Stop_Timeout();
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
// 	ID++;
// 	ID = ID > 0x09 ? 0x00 : ID;
// 	escreveID();
// 	
// 	// Zerar Timer
// 	TCNT3 = 0;
	
	ID = ID_mem;
	escreveID();
	
	PWM_Stop_Timeout();
	//desligaDisplay();
}

// void PWM_Start_Timeout() {
// 	// Configura prescaler para 8 e inicia timer
// 	TCCR3B |= (1 << CS31);
// }
// 
// void PWM_Stop_Timeout() {
// 	TCCR3B &= ~(1 << CS31);
// 	
// 	// Zerar Timer
// 	TCNT3 = 0;
// }
// 
// ISR(TIMER3_OVF_vect) {
// 	//ToDo:: Alterar percentual do PWM, valor em OCR3C. Usar efeito fade.
// 	if (pwm_fade_time != pwm_temp_fade_time) {
// 		pwm_fade_time = pwm_temp_fade_time;
// 	}
// 	
// 	REG16_WriteOCR3C(pwm_fade_time);
// }
