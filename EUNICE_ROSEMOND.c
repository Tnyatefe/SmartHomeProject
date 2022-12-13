
#include <MKL25Z4.h>
#include <stdio.h>

#define MASK(X)  (1UL<<X)

#define ENABLE_WDOG_REFRESH    1

// PIN FOR THERMISTOR
#define Thermister_analog (20)             // PTE20

// PIN FOR LDR
#define LDR_analog  (23)                   //PTE23

//PINS FOR WATER LEVEL SENSOR
#define Water_analog (22)                  //PTE22

// PIN FOR PIR MOTION SENSOR
#define pirSENSOR (8)                      //PTD8

//PINS FOR BLUETOOTH WIRELESS CONTROL
#define RX	1		//PTA1
#define TX	2		//PTA2
#define OSR 15
#define BAUD_RATE  	9600
#define SYS_CLOCK	20971520u

// PINS FOR LCD
#define D0 (7)		// PTC7
#define D1 (0)		// PTC0
#define D2 (3)		// PTC2
#define D3 (4)		// PTC4
#define D4 (5)		// PTC5
#define D5 (6)		// PTC6
#define D6 (10)		// PT10
#define D7 (11)		// PTC11

#define RS (12)		// PTC12
#define E (13)		// PTC13

// PINS FOR  LEDS
#define BLUE_LED (1)   //PTD1
#define GREEN_LED (19) //PTB19
#define RED_LED (18)   //PTB18
#define YELLOW_EXT (9)  //PTB9
#define EXT_BLUE (16)   //PTC16
#define LIGHT_LED (12)   //PTA12


// PINS FOR SERVO MOTOR
#define output_servo (5)  	         //PTA5 TPM0_CH2 FOR FAN
#define PTB2 (2)		             // TPM2_CH0

#define garage_servo (31)            //PTE31   TPM0_CH4   FOR GARAGE DOOR
#define PTB3  (3)                    //TPM2_CH1

//PINS FOR ULTRASONIC SENSOR
#define PTB0		(0)		// TPM1 CH0	for PWM trigger.
#define PTB1		(1)		//TPM1 CH1  for input capture on rising & falling

int distance=0;


// PINS FOR SEVEN SEGMENT DISPLAY

#define PIN_A (0)        //PTD0
#define PIN_B (7)        //PTD7
#define PIN_C (2)        //PTD2
#define PIN_D (3)        //PTD3
#define PIN_E (4)        //PTD4
#define PIN_F (5)        //PTD5
#define PIN_G (6)       //PTD6


// FUNCTION FOR THERMISTER
void initiate_ADC_TEMP(void) ;
void Sensor_value(void);


void detect_MOTION();
void initiate_Comparator(void);


// FUNCTION FOR LCD
void Init_LCDPins();
void lcd_init(void);
void delay_ms(int );
extern void SysTick_Handler();
unsigned long millis();
volatile unsigned long counter =0;
void display_temperature(void);

//FUNCTION FOR ONBOARD BLUE LED
void initialize_LEDPINs(void);
void default_blue(void);
void TEMP_PLUS_MOTION_SIGNAL();
void ULTRASONIC_OPEN_GARRAGE_SIGNAL();

//FUNCTION FOR TIMER
void init_LEDTimer();
void init_SERVOTimer();
void init_ULTRASONICTimer();


void TURN_ON();
//FUNCTION FOR SERVO MOTOR
void turn_fan_on(void);
void init_GARAGETimer();

//FUNCTIONS FOR SEVEN SEGMENT DISP
void display_doorCLOSE();
void display_doorOPEN();

//FUNCTIONS FOR BLUETOOTH
void initiate_UART(void);
void Control_Lights(unsigned int light1, unsigned int light2);
void lightLED(void);


char buff[20];
char Light_buff[20];
char Waterlevel_buff[20];
volatile int temp_readingADC=0;	          //the value reading the ADC
volatile int LIGHT_intensity=0;	          //the value reading the ADC
volatile int Water_level = 0;             // the value reading the ADC
int Temp=0;
int Light =0;
int pirMOTION_status = 0;

volatile char rxChar;
static int ctr=0;
static int ctrr=0;



int main (void) {
	initiate_ADC_TEMP();
	ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0);

	Init_LCDPins();
	lcd_init();
	initialize_LEDPINs();
	initiate_Comparator();
	init_LEDTimer();
	init_SERVOTimer();
	init_ULTRASONICTimer();
	init_GARAGETimer();
	initiate_UART();



	while (1) {

		display_temperature();
		lightLED();
        SIM->SRVCOP = 0x55;
        SIM->SRVCOP = 0xAA;

	}
}

/**************************************************PIR SENSOR INPUT WITH INTERRUPT***************************************************************/
void PORTD_IRQHandler(void){

		if(PORTD->ISFR & MASK(pirSENSOR)) {

			pirMOTION_status=1;
			PORTD->ISFR |= MASK(pirSENSOR);
		}

		else
			pirMOTION_status=0;
	}


/*****************************INITIALIZATION OF PINS **************************************************************/

void initialize_LEDPINs(void){
	SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK |  SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTA_MASK |SIM_SCGC5_PORTE_MASK |SIM_SCGC5_PORTC_MASK;

	__disable_irq();

	// Setup pin for pir sensor
	PORTD->PCR[pirSENSOR] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[pirSENSOR] |= PORT_PCR_MUX(1);

	PTD->PDDR &= ~MASK(pirSENSOR) ;


	PORTD->PCR[pirSENSOR] &=~PORT_PCR_IRQC_MASK;
	PORTD->PCR[pirSENSOR]  |= PORT_PCR_IRQC(12);

	PORTD->ISFR |= MASK(pirSENSOR);

	NVIC->ISER[0] |= MASK(PORTD_IRQn);
	__enable_irq();

	// Setup pins for LDR

	PORTE->PCR[LDR_analog] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[LDR_analog] |= PORT_PCR_MUX(5);

	//Setup pin for LEDS

	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);

	PTD->PDDR |= MASK(BLUE_LED) ;
	PTD->PSOR =MASK(BLUE_LED);

	PORTB->PCR[YELLOW_EXT] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[YELLOW_EXT] |= PORT_PCR_MUX(1);

	PTB->PDDR |= MASK(YELLOW_EXT) ;
	PTB->PCOR =MASK(YELLOW_EXT);

	PORTC->PCR[EXT_BLUE] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[EXT_BLUE] |= PORT_PCR_MUX(1);

	PTC->PDDR |= MASK(EXT_BLUE) ;
	PTC->PCOR =MASK(EXT_BLUE);

	PORTA->PCR[LIGHT_LED] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[LIGHT_LED] |= PORT_PCR_MUX(1);

	PTA->PDDR |= MASK(LIGHT_LED) ;
	PTA->PCOR =MASK(LIGHT_LED);

	// Setup pins for Seven segment display
	PORTD->PCR[PIN_A] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_A] |= PORT_PCR_MUX(1);

	PORTD->PCR[PIN_B] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_B] |= PORT_PCR_MUX(1);


	PORTD->PCR[PIN_C] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_C] |= PORT_PCR_MUX(1);

	PORTD->PCR[PIN_D] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_D] |= PORT_PCR_MUX(1);

	PORTD->PCR[PIN_E] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_E] |= PORT_PCR_MUX(1);

	PORTD->PCR[PIN_F] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_F] |= PORT_PCR_MUX(1);

	PORTD->PCR[PIN_G] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_G] |= PORT_PCR_MUX(1);

	PTD->PDDR |= MASK(PIN_A) ;
	PTD->PDDR |= MASK(PIN_B) ;
	PTD->PDDR |= MASK(PIN_C) ;
	PTD->PDDR |= MASK(PIN_D) ;
	PTD->PDDR |= MASK(PIN_E) ;
	PTD->PDDR |= MASK(PIN_F) ;
	PTD->PDDR |= MASK(PIN_G) ;

	PTD->PCOR |= MASK(PIN_A) ;
	PTD->PCOR |= MASK(PIN_B) ;
	PTD->PCOR |= MASK(PIN_C) ;
	PTD->PCOR |= MASK(PIN_D) ;
	PTD->PCOR |= MASK(PIN_E) ;
	PTD->PCOR |= MASK(PIN_F) ;
	PTD->PCOR |= MASK(PIN_G) ;

    //Setup pins for servo motors
	PORTA->PCR[output_servo ] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[output_servo ] |= PORT_PCR_MUX(3);

	PORTE->PCR[garage_servo ] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[garage_servo ] |= PORT_PCR_MUX(3);


	PORTB->PCR[PTB2 ] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB3 ] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3] |= PORT_PCR_MUX(3);

	// Setup pins for Ultrasonic sensor
	//set up pin PTB1 for PWM
	PORTB->PCR[PTB1] &=~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1] |=PORT_PCR_MUX(3);

	//set up pin PTB0 for input capture
	PORTB->PCR[PTB0] &=~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0] |=PORT_PCR_MUX(3);

    //Setup pins for Bluetooth
	PORTA->PCR[RX] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[RX] |=  PORT_PCR_MUX(2);
	PORTA->PCR[TX] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[TX] |=  PORT_PCR_MUX(2);

}

/**************************************Default pattern********************************************************************/
void default_blue(void){
	enum BLUE_states {BLUEON1,DELAYONE, BLUEOFF1,DELAYTWO,BLUEON2,DELAYTHREE,BLUEOFF2, DELAYFOUR};
	    static enum BLUE_states next_state = BLUEON1;
	    unsigned long current_time = 0u;
	    static unsigned long last_run = 0u;
	    unsigned long short_interval = 125;
	    unsigned long long_interval = 100;

	    	switch(next_state){

	    	case BLUEON1:
	    		PTD->PCOR=MASK(BLUE_LED);    // Blue on
	    		next_state = DELAYONE;
	    		break;

	    	case DELAYONE:
	    		current_time = millis();
	    		if((current_time - last_run) >= short_interval){
	    		last_run = current_time;
	    		next_state = BLUEOFF1;
	    		}
	    	   break;

	    	case BLUEOFF1:
	    		PTD->PSOR=MASK(BLUE_LED);    // Blue oFF
	    		next_state = DELAYTWO;
	    		break;

	    	case DELAYTWO:
	    		current_time = millis();
	    		if((current_time - last_run) >= short_interval){
	    		 last_run = current_time;
	    		  next_state = BLUEON2;
	    		}
	    		  break;

	    	case BLUEON2:
	    		PTD->PCOR=MASK(BLUE_LED);    // Blue on
	    		next_state = DELAYTHREE;
	    		break;

	    	case DELAYTHREE:
	    		current_time = millis();
	    		if((current_time - last_run) >= short_interval){
	    		 last_run = current_time;
	    		 next_state = BLUEOFF2;
	    		  }
	    	break;

	    	case BLUEOFF2:
	    		PTD->PSOR=MASK(BLUE_LED);    // Blue oFF
	    		next_state = DELAYFOUR;
	    		break;

	    	case DELAYFOUR:
	    		current_time = millis();
	    		 if((current_time - last_run) >= long_interval){
	    		 last_run = current_time;
	    		 next_state = BLUEON1;
	    		  }
	    		 break;

	    	default:
	    		next_state = BLUEON1;
	    		break;
	    }
	    }


/*********************************************************Initialization for TPM**********************************************/
void init_LEDTimer(){
	SIM->SCGC6 |=SIM_SCGC6_TPM0_MASK;

	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	TPM0->MOD= 262144 ;
	TPM0->SC |= TPM_SC_TOF_MASK;
	TPM0->SC |= TPM_SC_PS(3);

	TPM0->SC |= TPM_SC_CMOD(1);
}

/********************************************************Heartbit signals *****************************************************/
void TEMP_PLUS_MOTION_SIGNAL(){
	        enum yellow_states {ON1,DELAYONE, OFF1,DELAYTWO,ON2,DELAYTHREE};
		    static enum yellow_states next_state = ON1;

		    switch(next_state){

		    	    	case ON1:
		    	    		PTB->PSOR=MASK(YELLOW_EXT);    // Blue on
		    	    		next_state = DELAYONE;
		    	    		break;

		    	    	case DELAYONE:
		    	    		if(TPM0->SC & TPM_SC_TOF_MASK){
		    	    		TPM0->SC |= TPM_SC_TOF_MASK;}	//clear the flag
		    	    		next_state = OFF1;

		    	    	   break;

		    	    	case OFF1:
		    	    		PTB->PCOR=MASK(YELLOW_EXT);    // Blue oFF
		    	    		next_state = DELAYTWO;
		    	    		break;

		    	    	case DELAYTWO:
		    	    		if(TPM0->SC & TPM_SC_TOF_MASK){
		    	    		TPM0->SC |= TPM_SC_TOF_MASK;}	//clear the flag
		    	    		next_state = ON2;

		    	    		break;

		    	    	case ON2:
		    	    		PTB->PSOR=MASK(YELLOW_EXT);    // Blue on
		    	    		next_state = DELAYTHREE;
		    	    		break;

		    	    	case DELAYTHREE:
		    	    		if(TPM0->SC & TPM_SC_TOF_MASK){
		    	    		TPM0->SC |= TPM_SC_TOF_MASK;}	//clear the flag
		    	    		next_state = ON1;
		    	    	     break;


		    	    	default:
						next_state = ON1;
						break;

}
}


void ULTRASONIC_OPEN_GARRAGE_SIGNAL(){
	        enum blue_states {ON1,DELAYONE, OFF1,DELAYTWO,ON2,DELAYTHREE};
		    static enum blue_states next_state = ON1;

		    switch(next_state){

		    	    	case ON1:
		    	    		PTC->PSOR=MASK(EXT_BLUE);    // Blue on
		    	    		next_state = DELAYONE;
		    	    		break;

		    	    	case DELAYONE:
		    	    		if(TPM0->SC & TPM_SC_TOF_MASK){
		    	    		TPM0->SC |= TPM_SC_TOF_MASK;}	//clear the flag
		    	    		next_state = OFF1;

		    	    	   break;

		    	    	case OFF1:
		    	    		PTC->PCOR=MASK(EXT_BLUE);    // Blue oFF
		    	    		next_state = DELAYTWO;
		    	    		break;

		    	    	case DELAYTWO:
		    	    		if(TPM0->SC & TPM_SC_TOF_MASK){
		    	    		TPM0->SC |= TPM_SC_TOF_MASK;}	//clear the flag
		    	    		next_state = ON2;

		    	    		break;

		    	    	case ON2:
		    	    		PTC->PSOR=MASK(EXT_BLUE);    // Blue on
		    	    		next_state = DELAYTHREE;
		    	    		break;

		    	    	case DELAYTHREE:
		    	    		if(TPM0->SC & TPM_SC_TOF_MASK){
		    	    		TPM0->SC |= TPM_SC_TOF_MASK;}	//clear the flag
		    	    		next_state = ON1;
		    	    	     break;


		    	    	default:
						next_state = ON1;
						break;

}
}


/**************************************************ADC CONVERSIONS****************************************************************/
void initiate_ADC_TEMP(void) {

	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[Thermister_analog] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[Thermister_analog] |= PORT_PCR_MUX(0);

	PORTE->PCR[LDR_analog] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[LDR_analog] |= PORT_PCR_MUX(0);

	//16 bit single-ended conversion
	ADC0->CFG1 |= ADC_CFG1_MODE(3);
	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK ;


	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_SetPriority(ADC0_IRQn, 3);
	NVIC_EnableIRQ(ADC0_IRQn);
}




void ADC0_IRQHandler(void){
	if (ADC0->SC1[0] & ADC_SC1_COCO_MASK){
		temp_readingADC = ADC0->R[0];
		LIGHT_intensity = ADC0->R[0];
		ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0);
	}
}

void Display_sensedValues(void){


	float Temperature = (float)50.0*temp_readingADC/0xffff;
	Temp = Temperature;
	sprintf(buff,"Temperature= %d", Temp);
	printf("%s\n", buff);


	float Light_intensity = (float)100.0*LIGHT_intensity/0xffff;
	Light = Light_intensity;
	sprintf(Light_buff,"Light= %d", Light);
	printf("%s\n", Light_buff);

    float Water_LEVEL = (float)500.0*Water_level/0xffff;
	int LEVEL= Water_LEVEL;
	sprintf(Waterlevel_buff,"H20= %d",LEVEL);
	printf("%s\n", Waterlevel_buff);

}



/*********************************************************USING COMPARE WITH WATER LEVEL*************************************************************/

void initiate_Comparator(void) {
	SIM->SCGC4 |= SIM_SCGC4_CMP_MASK;
	CMP0->CR1 = CMP_CR1_EN_MASK | CMP_CR1_OPE_MASK;
	CMP0->MUXCR = CMP_MUXCR_PSEL(5) | CMP_MUXCR_MSEL(7);
	CMP0->SCR = CMP_SCR_IEF_MASK | CMP_SCR_IER_MASK;
	CMP0->DACCR = CMP_DACCR_DACEN_MASK | CMP_DACCR_VOSEL(32);
	NVIC_SetPriority(CMP0_IRQn, 128);
	NVIC_ClearPendingIRQ(CMP0_IRQn);
	NVIC_EnableIRQ(CMP0_IRQn);
}

void CMP0_IRQHandler(void) {
	if (CMP0->SCR & CMP_SCR_CFR_MASK) {
		 PTA->PSOR =MASK(LIGHT_LED);
		CMP0->SCR |=CMP_SCR_CFR_MASK;
	} else if (CMP0->SCR & CMP_SCR_CFF_MASK) {
		 PTA->PCOR =MASK(LIGHT_LED);
		CMP0->SCR |=CMP_SCR_CFF_MASK;
	}
}



/*******************************************************ULTRASONIC SENSOR ************************************************************/

void init_ULTRASONICTimer(){
	SIM->SCGC6 |=SIM_SCGC6_TPM1_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1) ;
	TPM1->MOD= 8192;

	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) |TPM_CnSC_ELSB(1)  ;
	TPM1->CONTROLS[0].CnV |= 3 ;

	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_ELSA(1) |TPM_CnSC_ELSB(1)  ;
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK | TPM_CnSC_CHIE_MASK ;
	TPM1->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(7) | TPM_SC_TOIE_MASK  ;
	TPM1->SC |= TPM_SC_CMOD(1);

	NVIC_ClearPendingIRQ(TPM1_IRQn);
	NVIC_SetPriority(TPM1_IRQn, 3);
	NVIC_EnableIRQ(TPM1_IRQn);
}

void TPM1_IRQHandler(){
	static int count=0;
	static unsigned int previous=0;
	unsigned int current=0;
	static unsigned int interval=0;
		if (TPM1->STATUS & TPM_STATUS_CH1F_MASK){
			current=TPM1->CONTROLS[1].CnV;
			current |= (count <<16);
			interval = current-previous;
			previous=current;
			TPM1->CONTROLS[1].CnSC |=TPM_CnSC_CHF_MASK;
	}

	if (TPM1->SC & TPM_SC_TOF_MASK){
		count++;
		TPM1->SC |= TPM_SC_TOF_MASK ;
		if (!( count %10)){
			 distance=interval;
						if (interval>7373)
							printf("discard %d\n", distance);
						else
							printf("distance in mm=%d\n", distance);
		}
	}
}

/**********************************************************7 SEGMENT DISP*************************************************************/
void display_doorOPEN(){
    PTD->PSOR |=MASK(PIN_A);
    PTD->PSOR |=MASK(PIN_B);
    PTD->PSOR |=MASK(PIN_C);
    PTD->PSOR |=MASK(PIN_D);
    PTD->PSOR |=MASK(PIN_E);
    PTD->PSOR |=MASK(PIN_F);
    PTD->PCOR |=MASK(PIN_G);
}

void display_doorCLOSE(){
    PTD->PSOR |=MASK(PIN_A);
    PTD->PCOR |=MASK(PIN_B);
    PTD->PCOR |=MASK(PIN_C);
    PTD->PSOR |=MASK(PIN_D);
    PTD->PSOR |=MASK(PIN_E);
    PTD->PSOR |=MASK(PIN_F);
    PTD->PCOR |=MASK(PIN_G);
}

/******************************************************SERVO MOTOR FOR GARAGE DOOR****************************************************************/

void init_GARAGETimer(){
	SIM->SCGC6 |=SIM_SCGC6_TPM2_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	TPM2->MOD= 0xCCCD;

	TPM2->CONTROLS[1].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM2->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;
	TPM2->CONTROLS[1].CnV =0x147B;
	TPM2->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK  ;

	TPM2->SC |= TPM_SC_CMOD(1);

	NVIC_ClearPendingIRQ(TPM2_IRQn);
	NVIC_SetPriority(TPM2_IRQn, 3);
	NVIC_EnableIRQ(TPM2_IRQn);
}

void OPEN_GARAGE(){

	static enum stages{ stage1,stage2}next_stage=stage1;

	if(distance <=50) {
	ctrr++;
	if (ctrr>=200){
		ctrr=0;
		switch (next_stage){
		case stage1:
			TPM2->CONTROLS[1].CnV =0xA3D;
			next_stage=stage2;
			break;
		case stage2:
			TPM2->CONTROLS[1].CnV =0x147A;
			next_stage=stage1;
			break;
		default:
			TPM2->CONTROLS[1].CnV =0xA3D;
			next_stage=stage1;
			break;

		}
	}
	   ULTRASONIC_OPEN_GARRAGE_SIGNAL();
	   display_doorOPEN();

	}

	else{
	    default_blue();
	    PTC->PCOR=MASK(EXT_BLUE);    // Blue off
	    display_doorCLOSE();}

}


/***********************************************************SERVO MOTOR FOR FAN **********************************************/
void init_SERVOTimer(){
	SIM->SCGC6 |=SIM_SCGC6_TPM2_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	TPM2->MOD= 0xCCCD;

	TPM2->CONTROLS[0].CnSC |= TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1) ;
	TPM2->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
	TPM2->CONTROLS[0].CnV =0x147B;
	TPM2->SC |=  TPM_SC_TOF_MASK | TPM_SC_PS(3) | TPM_SC_TOIE_MASK  ;

	TPM2->SC |= TPM_SC_CMOD(1);

	NVIC_ClearPendingIRQ(TPM2_IRQn);
	NVIC_SetPriority(TPM2_IRQn, 3);
	NVIC_EnableIRQ(TPM2_IRQn);
}

void turn_fan_on(){
	static enum stages{ stage1,stage2}next_stage=stage1;
	if((pirMOTION_status) && (Temp >=25) ){
	ctr++;
	if (ctr>=200){
		ctr=0;
		switch (next_stage){
		case stage1:
			TPM2->CONTROLS[0].CnV =0xA3D;
			next_stage=stage2;
			break;
		case stage2:
			TPM2->CONTROLS[0].CnV =0x147A;
			next_stage=stage1;
			break;
		default:
			TPM2->CONTROLS[0].CnV =0xA3D;
			next_stage=stage1;
			break;

		}
	}
	 TEMP_PLUS_MOTION_SIGNAL();
	 PTD->PSOR=MASK(BLUE_LED);    // Blue oFF
	}

	else
		PTB->PCOR=MASK(YELLOW_EXT);    // Blue off
	    default_blue();

}




void TPM2_IRQHandler(){


	if (TPM2->SC & TPM_SC_TOF_MASK){
		turn_fan_on();
		OPEN_GARAGE();
	}
	TPM2->SC |= TPM_SC_TOF_MASK ;
}

/************************************************BLUETOOTH CONTROL WITH LDR****************************************************************/

void initiate_UART(void){
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
	uint8_t sbr = (uint16_t)((SYS_CLOCK)/((OSR+1) *BAUD_RATE ));
	UART0->BDH =0;
	UART0->BDL=sbr;
	UART0->C2  |= UART_C2_RIE_MASK | UART_C2_RE_MASK ;
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
}


void Control_Lights(unsigned int light1, unsigned int light2) {
	if (light1) {
		if (LIGHT_intensity <10){
			PTB->PCOR = MASK(GREEN_LED);
	} else {
			PTB->PSOR = MASK(GREEN_LED);
	}}
	if (light2) {
		if (LIGHT_intensity >=10){
			PTB->PCOR = MASK(RED_LED);
	}	else {
			PTB->PSOR = MASK(RED_LED);
	}}
}

void lightLED(void){
	switch (rxChar){
		case 'g': Control_Lights(1,0);
				break;
		case 'r': Control_Lights(0,1);
				break;
		case 'b': Control_Lights(1,1);
				break;
		default:
				Control_Lights(1,1);
	}
}


void UART0_IRQHandler(void) {
	uint8_t ch;

	if (UART0->S1 & (UART_S1_OR_MASK |UART_S1_NF_MASK |
		UART_S1_FE_MASK | UART_S1_PF_MASK)) {
			UART0->S1 |= UART0_S1_OR_MASK | UART0_S1_NF_MASK | UART0_S1_FE_MASK | UART0_S1_PF_MASK;
			ch = UART0->D;
	}
	if (UART0->S1 & UART0_S1_RDRF_MASK) {
		ch = UART0->D;
		rxChar=ch;
	}
}


/**********************************************LCD DISPLAY************************************************************************/
void Init_LCDPins(void) {
	// clock gate
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;;

	// Make C pins GPIO
	PORTC->PCR[D0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D0] |= PORT_PCR_MUX(1);
	PORTC->PCR[D1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D1] |= PORT_PCR_MUX(1);
	PORTC->PCR[D2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D2] |= PORT_PCR_MUX(1);
	PORTC->PCR[D3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D3] |= PORT_PCR_MUX(1);
	PORTC->PCR[D4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D4] |= PORT_PCR_MUX(1);
	PORTC->PCR[D5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D5] |= PORT_PCR_MUX(1);
	PORTC->PCR[D6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D6] |= PORT_PCR_MUX(1);
	PORTC->PCR[D7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[D7] |= PORT_PCR_MUX(1);

	PORTC->PCR[RS] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[RS] |= PORT_PCR_MUX(1);
	PORTC->PCR[E] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[E] |= PORT_PCR_MUX(1);


	// Set ports to outputs
	PTC->PDDR |= MASK(D0) | MASK(D1)  | MASK(D2)  | MASK(D3)  | MASK(D4)  | MASK(D5)  | MASK(D6)  | MASK(D7) ;
	PTC->PDDR |= MASK(RS) | MASK(E);

	SysTick->LOAD = (20971520u/1000u)-1 ;
		SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_ENABLE_Msk |SysTick_CTRL_TICKINT_Msk;
}

void delay_ms(int t_ms){
	for (int i=0; i<t_ms; i++)
		for (int j=0; j<48000;j++);
}

void write_D0_D7(unsigned char instruction){
		(instruction & MASK(0) ) ?  	(PTC->PSOR = MASK(D0) ): (PTC->PCOR = MASK(D0) );
		(instruction & MASK(1) ) ?  	(PTC->PSOR = MASK(D1) ): (PTC->PCOR = MASK(D1) );
		(instruction & MASK(2) ) ?  	(PTC->PSOR = MASK(D2) ): (PTC->PCOR = MASK(D2) );
		(instruction & MASK(3) ) ?  	(PTC->PSOR = MASK(D3) ): (PTC->PCOR = MASK(D3) );
		(instruction & MASK(4) ) ?  	(PTC->PSOR = MASK(D4) ): (PTC->PCOR = MASK(D4) );
		(instruction & MASK(5) ) ?  	(PTC->PSOR = MASK(D5) ): (PTC->PCOR = MASK(D5) );
		(instruction & MASK(6) ) ?  	(PTC->PSOR = MASK(D6) ): (PTC->PCOR = MASK(D6) );
		(instruction & MASK(7) ) ?  	(PTC->PSOR = MASK(D7) ): (PTC->PCOR = MASK(D7) );
}


void lcd_write_instruc (unsigned char instruction){
	delay_ms(2);
	PTC->PCOR = MASK(RS);
	PTC->PCOR= MASK(E);
	write_D0_D7(instruction);
	PTC->PSOR= MASK(E);
	delay_ms(2);
	PTC->PCOR= MASK(E);
	}


void lcd_write_char (unsigned char c){

	delay_ms(2);
	PTC->PSOR = MASK(RS);
	PTC->PCOR= MASK(E);
	write_D0_D7(c);
	PTC->PSOR= MASK(E);
	delay_ms(2);//<---
	PTC->PCOR= MASK(E);

	}

void lcd_init(void)
{
	delay_ms(2);
	lcd_write_instruc(0x06);
	lcd_write_instruc(0x0C);
	lcd_write_instruc(0x38);
}

void lcd_clear(void)
	{
	delay_ms(2);
	lcd_write_instruc(0x01);
	lcd_write_instruc(0x02);
}


void lcd_goto(unsigned char column, unsigned char row)
	{
	delay_ms(2);
	if(row==0)
	lcd_write_instruc(0x80 + column);
	if(row==1)
	lcd_write_instruc(0xC0+ column);
	}


void lcd_write_string(char *s)
	{
	delay_ms(2);
	while(*s != 0)
	{
	lcd_write_char(*s);
	s++;
	}
	}

void display_temperature(){
	    unsigned long current_time = 0u;
		static unsigned long last_run = 0u;

		current_time = millis();
		if((current_time - last_run) >= 1){
		last_run = current_time;

		lcd_clear();
	    lcd_goto(0,0);
		lcd_write_string("TC");
		lcd_goto(0,2);

		char displayTemp[16];
		sprintf(displayTemp,":%dC ",Temp);
		lcd_write_string(displayTemp);

		lcd_goto(0,8);
	    lcd_write_string("D:");

	    lcd_goto(0,11);

	    char displaydist[16];
	    sprintf(displaydist,"%dmm",distance);
	    lcd_write_string(displaydist);
		}

}


unsigned long millis(void){
	return (unsigned long)counter ;

}









