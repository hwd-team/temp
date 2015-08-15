/*******************************************************************************************
  SIMPL+ Module Information
  (Fill in comments below)
*******************************************************************************************/
/*
Dealer Name:
System Name:
System Number:
Programmer:
Comments:
*/

/*******************************************************************************************
  Compiler Directives
  (Uncomment and declare compiler directives as needed)
*******************************************************************************************/
// #ENABLE_DYNAMIC
// #SYMBOL_NAME ""
// #HINT ""
//#DEFINE_CONSTANT	DEBUG		1
#DEFINE_CONSTANT	DEBUG2		1
#DEFINE_CONSTANT	TIPO		0x01
#DEFINE_CONSTANT	TO_BRIDGE	0x8F
#DEFINE_CONSTANT	TO_GO		0x90
#DEFINE_CONSTANT	C_ON		0xF0
#DEFINE_CONSTANT	C_OFF		0x0F
#DEFINE_CONSTANT	strErr		65535
#DEFINE_CONSTANT	SET_ID		0x11
#DEFINE_CONSTANT	SET_SCAN	0x12
#DEFINE_CONSTANT    VIRTUAL_CHECKSUM 0xAA
                                       
#DEFINE_CONSTANT	QDE_MODU	24                                       
#DEFINE_CONSTANT	QDE_CIRC	192	// 8*QDE_MODU
#DEFINE_CONSTANT	CH8			8

//#DEFINE_CONSTANT	DEBUG		1
// #CATEGORY "" 
// #PRINT_TO_TRACE
// #DIGITAL_EXPAND 
// #ANALOG_SERIAL_EXPAND 
// #OUTPUT_SHIFT 
// #HELP_PDF_FILE ""
#DEFAULT_VOLATILE
#ENABLE_STACK_CHECKING
#ENABLE_TRACE
// #ENCODING_ASCII
// #ENCODING_UTF16
// #ENCODING_INHERIT_FROM_PARENT
// #ENCODING_INHERIT_FROM_PROGRAM
/*
#HELP_BEGIN
   (add additional lines of help lines)
#HELP_END
*/

/*******************************************************************************************
  Include Libraries
  (Uncomment and include additional libraries as needed)
*******************************************************************************************/
// #CRESTRON_LIBRARY ""
                                                               
/*******************************************************************************************
  DIGITAL, ANALOG and SERIAL INPUTS and OUTPUTS
  (Uncomment and declare inputs and outputs as needed)
*******************************************************************************************/
DIGITAL_INPUT _SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_;
DIGITAL_INPUT	Rx_TIMEDOUT,_SKIP_,EN_SCAN,DIS_SCAN,_SKIP_,ADD_ID[QDE_MODU],_SKIP_,REM_ID[QDE_MODU],_SKIP_,LIGA[QDE_CIRC],_SKIP_,DESLIGA[QDE_CIRC],_SKIP_;
BUFFER_INPUT	Rx$[800];

DIGITAL_OUTPUT _SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_,_SKIP_;                      
DIGITAL_OUTPUT	START_Rx_TIMEOUT,_SKIP_,_SKIP_,_SKIP_,_SKIP_;                        
DIGITAL_OUTPUT	FB[QDE_CIRC],_SKIP_,PUL[QDE_CIRC,QDE_CIRC];                        
STRING_OUTPUT	_SKIP_,Tx$;


/*******************************************************************************************
  Parameters
  (Uncomment and declare parameters as needed)
*******************************************************************************************/
INTEGER_PARAMETER	ID[QDE_MODU,QDE_MODU];


/*******************************************************************************************
  Parameter Properties
  (Uncomment and declare parameter properties as needed)
*******************************************************************************************/
#BEGIN_PARAMETER_PROPERTIES ID
   propValidUnits = unitHex;
   propDefaultUnit = unitHex;
   propBounds = 0x00 , 0x1E;
#END_PARAMETER_PROPERTIES


/*******************************************************************************************
  Global Variables
  (Uncomment and declare global variables as needed)
  Note:  Be sure to initialize all declared STRING variables as needed
         For example, in Function Main: myString = "";
*******************************************************************************************/
//RingBuff buff;    
integer in_char;
integer bytes[10], count;


/*******************************************************************************************
  Functions
  (Add any additional functions here)
  Note:  Functions must be physically placed before the location in
         the code that calls them.
*******************************************************************************************/
FUNCTION ESRL8_MAKE_485_STR(INTEGER _ID, INTEGER PARAM_2, INTEGER PARAM_3)
{
	INTEGER CHECK_SUM, i;
	INTEGER BYTES[10];
	STRING STR$[10];
	
	BYTES[0] = 0xFF;
	BYTES[1] = 0x05;
	BYTES[2] = _ID;
	BYTES[3] = TIPO;
	BYTES[4] = PARAM_2;
	BYTES[5] = PARAM_3;
	BYTES[6] = 0x00;
	BYTES[7] = 0x00;
	
//	CHECK_SUM = BYTES[0];
//	FOR (i = 1 TO 7) {
//		CHECK_SUM = CHECK_SUM ^ BYTES[i];
//	}
	
//	BYTES[8] = CHECK_SUM;

	BYTES[8] = VIRTUAL_CHECKSUM;
	BYTES[9] = 0x0D;
	
	//	Enviar comando
	STR$ = "";
	FOR (i = 0 TO 9) {
		MAKESTRING(STR$, "%s%c", STR$, BYTES[i]); 
	} 
	
	Tx$ = STR$;
}

FUNCTION ESRL8_MAKE_232_FULL_STR(integer cmd1, integer cmd2, integer param_1, integer param_2, integer param_3)
{
	INTEGER CHECK_SUM, i;
	INTEGER BYTES[10];
	STRING STR$[10];
	
	BYTES[0] = 0x05;
	BYTES[1] = cmd1;
	BYTES[2] = cmd2;
	BYTES[3] = param_1;
	BYTES[4] = param_2;
	BYTES[5] = param_3;
	
//	CHECK_SUM = BYTES[0];
//	FOR (i = 1 TO 5) {
//		CHECK_SUM = CHECK_SUM ^ BYTES[i];
//	}
	
//	BYTES[6] = CHECK_SUM;

	BYTES[6] = VIRTUAL_CHECKSUM;
	BYTES[7] = 0x0D;
	
	//	Enviar comando
	STR$ = "";
	FOR (i = 0 TO 7) {
		MAKESTRING(STR$, "%s%c", STR$, BYTES[i]); 
	} 
	
	Tx$ = STR$;
}

FUNCTION ESRL8_MAKE_232_STR(integer _id, integer param_2, integer param_3) {
	ESRL8_MAKE_232_FULL_STR(TO_GO, TIPO, _id, param_2, param_3);
}

function limpa_bytes() {
	integer i;
	
	for (i = 0 to 10) {
		bytes[i] = 0;
	}
	
	count = 0;
}

function trata_string() {
	integer i, valido, check_sum, ch_ini;

	valido = 1;
	check_sum = 0;
	
	#if_defined DEBUG2 print("0x%02X:", bytes[2]);		#endif

	FOR (i = 0 TO 9) {
		#if_defined DEBUG2 print(" %02X", bytes[i]);		#endif
			
		IF (i = 0) { /*check_sum = bytes[i];*/ check_sum = VIRTUAL_CHECKSUM; }
		//ELSE IF (i <= 7) { check_sum = check_sum ^ bytes[i]; }
		
		SWITCH (i) {
			CASE (0): { IF (bytes[i] = 0xFF)	{ CONTINUE; } ELSE { #if_defined DEBUG PRINT("0x%02X Inicio invalido: 0x%02X\n",	bytes[2], bytes[i]);	#endif valido = 0; i = 10; } }
			CASE (1): { IF (bytes[i] = 0x06)	{ CONTINUE; } ELSE { #if_defined DEBUG PRINT("0x%02X Identif. invalido: 0x%02X\n",	bytes[2], bytes[i]);	#endif valido = 0; i = 10; } }
			CASE (2): { CONTINUE; }			// ID
			CASE (3): { IF (bytes[i] = TIPO) 	{ CONTINUE; } ELSE { #if_defined DEBUG PRINT("0x%02X Tipo invalido: 0x%02X\n",		bytes[2], bytes[i]);	#endif valido = 0; i = 10; } }
			CASE (4): { CONTINUE; }			// STATUS PULSADORES LOWER
			CASE (5): { CONTINUE; }			// STATUS PULSADORES UPPER
			CASE (6): { CONTINUE; }			// STATUS RELES LOWER
			CASE (7): { CONTINUE; }			// STATUS RELES LOWER
			CASE (8): { CONTINUE; }			// CHECKSUM
			CASE (9): { IF (bytes[i] = 0x0D)	{ CONTINUE; } ELSE { #if_defined DEBUG2 PRINT("0x%02X Final invalido 0x%02X\n", bytes[2], bytes[i]);		#endif i = 10; } }
			DEFAULT:  { PRINT("0x%02X String mal formada\n", bytes[2]); }
		}
	}
	#if_defined DEBUG2 print("\n");		#endif
	
	IF (valido = 1) {
		IF (bytes[8] = check_sum) {
			ch_ini = (bytes[2] - 1)*CH8;
			
			FOR (i = 0 TO 3) {
				// PULSADORES LOWER
				IF ( bytes[4] & (1 << i) )	{	PUL[ch_ini + i + 1] = 1; }
				ELSE 						{	PUL[ch_ini + i + 1] = 0; }
				
				// PULSADORES UPPER
				IF ( bytes[5] & (1 << i) ) 	{	PUL[(ch_ini + i + 1) + 4] = 1; }
				ELSE 						{	PUL[(ch_ini + i + 1) + 4] = 0; }
				
				// RELES LOWER
				IF ( bytes[6] & (1 << i) ) 	{	FB[ch_ini + i + 1] = 1; }
				ELSE 						{	FB[ch_ini + i + 1] = 0; }
				
				// RELES UPPER
				IF ( bytes[7] & (1 << i) ) 	{	FB[(ch_ini + i + 1) + 4] = 1; }
				ELSE 						{	FB[(ch_ini + i + 1) + 4] = 0; }
			}
		}
		ELSE {
			#if_defined DEBUG2 PRINT("0x%02X Checksum invalido. %02X != %02X", bytes[2], bytes[8], check_sum);	#endif
		}
	}
}

/*******************************************************************************************
  Event Handlers
  (Uncomment and declare additional event handlers as needed)
*******************************************************************************************/ 
RELEASE Rx_TIMEDOUT {
	// Resetar Buffer Circular de recepcao de dados
	//RingBuffInit(buff);
}

push ADD_ID {
	integer index;	
	index = GetLastModifiedArrayIndex();
	
	ESRL8_MAKE_232_FULL_STR(TO_BRIDGE, SET_ID, 0x01, ID[index], TIPO);
}

push REM_ID {
	integer index;
	index = GetLastModifiedArrayIndex();
	
	ESRL8_MAKE_232_FULL_STR(TO_BRIDGE, SET_ID, 0x00, ID[index], TIPO);
}

PUSH LIGA {
	INTEGER index;
	integer quo, res, _id;
	
	index = GetLastModifiedArrayIndex();
	
	quo = index/8;
	res = index % 8;
	
	if (res > 0) {
		_id = quo + 1;
		index = res;
	}
	else {
		//_id = quo - 1;
		_id = quo;
		index = 8;
	}

	ESRL8_MAKE_232_STR(_id, C_ON, 1 << (INDEX - 1));
}

PUSH DESLIGA {
	INTEGER index;
	integer quo, res, _id;
	
	index = GetLastModifiedArrayIndex();
	
	quo = index/8;
	res = index % 8;
	
	if (res > 0) {
		_id = quo + 1;
		index = res;
	}
	else {
		//_id = quo - 1;
		_id = quo;
		index = 8;
	}
	
	ESRL8_MAKE_232_STR(_id, C_OFF, 1 << (INDEX - 1));
} 

threadsafe
change Rx$ {
	//integer bytes[10], count;
	
	in_char = 0;
	
	while (in_char <> 65535) {
		in_char = getc(Rx$);
		
		if (count = 0) {
			if (in_char = 0xFF) {
				bytes[count] = in_char;
				count = count + 1;
			}
			else {
				limpa_bytes();
				#if_defined DEBUG	print("0x%02X Primeiro char invalido: %02X\n", 0, in_char);	#endif
			}
		}
		else if (count = 1) {
			if (in_char = 0x06) {
				bytes[count] = in_char;
				count = count + 1;
			}
			else {
				limpa_bytes();
				#if_defined DEBUG	print("0x%02X Segundo char invalido: %02X\n", 0, in_char);		#endif
			}
		}
		else if (count > 1 && count < 10) {
			bytes[count] = in_char;
			count = count + 1;
			
			if (count = 10) {
				if (bytes[0] = 0xFF && bytes[1] = 0x06 && bytes[9] = 0x0D) {
					#if_defined DEBUG	print("0x%02X Complete\n", 0);		#endif
					trata_string();
					limpa_bytes();
				}
				else {
					limpa_bytes();
					#if_defined DEBUG	print("0x%02X String invalida\n", 0);		#endif
				}
			}
		}
		else {
			limpa_bytes();
			#if_defined DEBUG		print("0x%02X count >= 10\n", 0);		#endif
		}
		
	}
}

PUSH EN_SCAN {
	ESRL8_MAKE_232_FULL_STR(TO_BRIDGE, SET_SCAN, 0x01, 0x00, 0x00);
}

PUSH DIS_SCAN {
	ESRL8_MAKE_232_FULL_STR(TO_BRIDGE, SET_SCAN, 0x00, 0x00, 0x00);
}

/*******************************************************************************************
  Main()
  Uncomment and place one-time startup code here
  (This code will get called when the system starts up)
*******************************************************************************************/
Function Main()
{
    WaitForInitializationComplete();
	limpa_bytes();
}
                             
