/// BAREMETAL ///

// RGB
uint32_t ui32Color[3];
// Establece como reloj del sistema el PLL (200Mhz, alimentado por el XTAL de 16MHz), dividido por 4--> 50MHz
// Indica las 3 componentes del color a sacar: prueba a cambiarlas para diferentes colores
ui32Color[RED] = 0x7FFF;   // Minimo: 0; Maxima 0xFFFF
ui32Color[BLUE] = 0x7FFF; // Minimo: 0; Maxima 0xFFFF
ui32Color[GREEN] = 0x0;       // Minimo: 0; Maxima 0xFFFF
SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
RGBInit(0);             // Inicializa el sistema de control de LEDs
RGBColorSet(ui32Color); // Saca el color indicado en el array por los LEDs
RGBIntensitySet(0.5);   // Reduce la intensidad al 50% del maximo
// RGBSet(ui32Color,0.5); // Esta instruccion equivale a las dos anteriores
RGBEnable();              // Habilita la generacion PWM para el encendido de los LEDs
//RGBBlinkRateSet(1.0f);  // Parpadeo del LED a 1Hz. Hay que definir la posicion del WTIMER5B de la tabla de
// vectores con la RTI de API "RGBBlinkIntHandler"

// buttons intterrupt
void configButtonsINTR(){
           
    // config buttons                                     
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_GPIOF );    
    ButtonsInit();
    GPIOIntClear(   GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); 
    GPIOIntTypeSet( GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);  // *    
    GPIOIntEnable(  GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);    
        IntRegister(    INT_GPIOF, Buttons_ISR );   
        IntEnable(      INT_GPIOF );     
        IntMasterEnable();
}

#define DBOUNCE         10

void Buttons_ISR(void){
    static int last_left = -DBOUNCE, last_right = -DBOUNCE;
    int time = xTaskGetTickCountFromISR();
    int intStatus = GPIOIntStatus(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    int pinStatus = GPIOPinRead(  GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    BaseType_t xHigherPriorityTaskWoken;

    if(intStatus & LEFT_BUTTON)
        if ( (time-last_left) > DBOUNCE ){
            last_left = time;

        }

    if(intStatus & RIGHT_BUTTON)
        if ( (time-last_right) > DBOUNCE ){
            last_right = time;

        }

    GPIOIntClear( GPIO_PORTF_BASE, intStatus);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// interrupt
void configInINTR(){
    // config interrupt                          //Check
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOl); // *
    GPIOPinTypeGPIOInput(GPIO_PORTl_BASE, GPIO_PIN_n);  // * *
    GPIOIntClear(        GPIO_PORTl_BASE, GPIO_PIN_n);  // * *
    GPIOIntTypeSet(      GPIO_PORTl_BASE, GPIO_PIN_n, GPIO_FALLLING_EDGE);  // * * * 
    GPIOIntEnable(       GPIO_PORTl_BASE, GPIO_PIN_n);  // * *
        IntRegister(INT_GPIOl, voidGPIOl_ISR);  // *
        IntEnable(  INT_GPIOl);                 // *
        IntMasterEnable();
}

// pwm
void configPWM(){
   // config PORT
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOl);            // *
    GPIOPinTypeGPIOOutput( GPIO_PORTl_BASE, GPIO_PIN_n);    // * *
    GPIOPinWrite(          GPIO_PORTl_BASE, GPIO_PIN_n, 0); // *
    GPIOPinConfigure(GPIO_PF1_TnCCPn);                      // **
    GPIOPinTypePWM(GPIO_PORTl_BASE, GPIO_PIN_n);            // * *

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMERn);         // * *
    TimerConfigure( TIMERn_BASE, TIMER_CFG_l_PWM | TIMER_CFG_SPLIT_PAIR);   // * *
    TimerLoadSet(   TIMERn_BASE, TIMER_l, ui32Period);    // * *
    TimerMatchSet(  TIMERn_BASE, TIMER_l, ui32DutyCycle); // * *
    TimerEnable(    TIMERn_BASE, TIMER_l);  // * *
}


void Timer0IntHandler(void)
{
	// Borra la interrupcion de Timer
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Lee el estado actual del LED y escribe el estado opuesto
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
	}
}

/// BOTH TIMERS
void configBothTimer(){
    uint32_t ui32Period = (SysCtlClockGet() / 10) / 2; // SysCtlClockGet() -> 1seg -> 1s/10/2 = 0.1/2 = 0.05s => 20Hz

    // Config TIMERn                                    //check
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMERn);       // *
    TimerConfigure(TIMERn_BASE, TIMER_CFG_PERIODIC);    // *

    TimerLoadSet(TIMERn_BASE, TIMER_A, ui32Period -1);  // * 
        IntRegister( INT_TIMERnA, voidTIMERnA_ISR);     // * * 
        IntEnable(   INT_TIMERnA );                     // *
    TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT );  // * 
    TimerIntEnable( TIMERn_BASE, TIMER_TIMA_TIMEOUT );  // * 
        IntMasterEnable();
    TimerEnable( TIMERn_BASE, TIMER_A );                // * 
}

void voidTIMERnA_ISR(void){								// *
	TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT );  // * 
}

TimerDisable(  TIMERn_BASE, TIMER_A );		         // * 
TimerLoadSet(  TIMERn_BASE, TIMER_A, ui32Period -1); // * * 
TimerIntClear( TIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(   TIMERn_BASE, TIMER_A );			     // * 


/// SPLIT TIMERS
void configSplitTimer(){
    uint16_t ui32PeriodA = ui32PeriodB = (SysCtlClockGet() / 10) / 2; // SysCtlClockGet() -> 1seg -> 1s/10/2 = 0.1/2 = 0.05s => 20Hz

    // Config TIMERn                                    //check
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMERn);       // *
    TimerConfigure(TIMERn_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);

    TimerLoadSet(TIMERn_BASE, TIMER_A, ui32PeriodA -1); // *
    TimerLoadSet(TIMERn_BASE, TIMER_B, ui32PeriodB -1); // *
        IntRegister( INT_TIMERnA, voidTIMERnA_ISR);     // * *
        IntRegister( INT_TIMERnB, voidTIMERnB_ISR);     // * *
        IntEnable(   INT_TIMERnA );                     // *
        IntEnable(   INT_TIMERnB );                     // *
    TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT );  // *
    TimerIntEnable( TIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT );  // *
        IntMasterEnable();
    TimerEnable( TIMERn_BASE, TIMER_A |  TIMER_B);      // *
}

void voidTIMERnA_ISR(void){							  // **
	TimerIntClear( TIMERn_BASE, TIMER_TIMA_TIMEOUT ); // * *
}

void voidTIMERnB_ISR(void){							  // **
	TimerIntClear( TIMERn_BASE, TIMER_TIMB_TIMEOUT ); // * *
}

TimerDisable(  TIMERn_BASE, TIMER_A );		         // * 
TimerLoadSet(  TIMERn_BASE, TIMER_A, ui64Period -1); // * * 
TimerIntClear( TIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(   TIMERn_BASE, TIMER_A );			     // * 
TimerDisable(  TIMERn_BASE, TIMER_B );		         // * 
TimerLoadSet(  TIMERn_BASE, TIMER_B, ui64Period -1); // * * 
TimerIntClear( TIMERn_BASE, TIMER_TIMB_TIMEOUT );    // *
TimerEnable(   TIMERn_BASE, TIMER_B );			     // * 


/// BOTH WIDE TIMERS
void configWideBothTimer(){
    uint64_t ui64Period = (SysCtlClockGet() * 1000);

    // Config TIMERn                                       //check
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMERn);         // *
    TimerConfigure(WTIMERn_BASE, TIMER_CFG_PERIODIC);      // *

    TimerLoadSet64(WTIMERn_BASE,  ui64Period -1);  // * * 
        IntRegister( INT_WTIMERnA, voidWTIMERnA_ISR);      // * * 
        IntEnable(   INT_WTIMERnA );                       // *
    TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // * 
    TimerIntEnable( WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // * 
        IntMasterEnable();
    TimerEnable( WTIMERn_BASE, TIMER_A );                 // * 
}

void voidWTIMERnA_ISR(void){							// *
	TimerIntClear( WTIMERn_BASE, TIMER_TIMA_TIMEOUT );  // * 
}

TimerDisable(   WTIMERn_BASE, TIMER_A );		       // * 
TimerLoadSet64( WTIMERn_BASE, ui64Period -1); // * * 
TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(    WTIMERn_BASE, TIMER_A );			   // * 


/// SPLIT WIDE TIMERS ///
void configSplitTimer(){
    uint16_t ui32PeriodA = ui32PeriodB = (SysCtlClockGet() / 10) / 2; // SysCtlClockGet() -> 1seg -> 1s/10/2 = 0.1/2 = 0.05s => 20Hz

    // Config WTIMERn                                     //check
    SysCtlPeripheralEnable(     SYSCTL_PERIPH_WTIMERn);  // *
    TimerConfigure(WTIMERn_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);
    TimerLoadSet(  WTIMERn_BASE, TIMER_A, ui32PeriodA -1); // *
    TimerLoadSet(  WTIMERn_BASE, TIMER_B, ui32PeriodB -1); // *
        IntRegister(   INT_WTIMERnA, voidWTIMERnA_ISR);  // * *
        IntRegister(   INT_WTIMERnB, voidWTIMERnB_ISR);  // * *
        IntEnable(     INT_WTIMERnA );                   // *
        IntEnable(	   INT_WTIMERnB );                   // *
    TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT );  // *
    TimerIntEnable( WTIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT );  // *
    	IntMasterEnable();
    TimerEnable(    WTIMERn_BASE, TIMER_A | TIMER_B );   // *
}

void voidWTIMERnA_ISR(void){						    // *
	TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT ); // *
}

void voidWTIMERnB_ISR(void){							// *
	TimerIntClear(  WTIMERn_BASE, TIMER_TIMB_TIMEOUT ); // *
}

TimerDisable(  WTIMERn_BASE, TIMER_A );		       // * 
TimerLoadSet(  WTIMERn_BASE, TIMER_A, ui32Period -1); // * * 
TimerIntClear( WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(   WTIMERn_BASE, TIMER_A );			   // * 
TimerDisable(  WTIMERn_BASE, TIMER_B );		       // * 
TimerLoadSet(  WTIMERn_BASE, TIMER_B, ui32Period -1); // * * 
TimerIntClear( WTIMERn_BASE, TIMER_TIMB_TIMEOUT );    // *
TimerEnable(   WTIMERn_BASE, TIMER_B );			   // * 

//Other interesting functions

TimerDisable();
TimerLoadGet();
TimerLoadGet64();
TimerControlTrigger();