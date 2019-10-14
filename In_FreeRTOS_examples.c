////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BASIC  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***********/
/** DELAY **/
/***********/

TickType_t xLastWakeTime;
xLastWakeTime = xTaskGetTickCount();
vTaskDelayUntil( (TickType_t*) &xLastWakeTime, (TickType_t) xTimeIncrement );

vTaskDelay( (TickType_t) xTicksToDelay );

xTaskAbortDelay( TaskHandle_t xTask );

vTaskPrioritySet( xTask, tskIDLE_PRIORITY + 1 );

vTaskSuspend( xTaskToSuspend ); // NULL suspends caller task
vTaskResume( xTaskToResume );
xTaskResumeFromISR( xTaskToResume );


IntPendSet(INT_I2C3);    //Produce un disparo software de la ISR (INT_GPIOF)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IPC  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************************/
/** EVENT GROUPS or FLAGS **/
/***************************/

//flags
#define _EV (1 << 0)
#define _EV (1 << )

//init
static EventGroupHandle_t xEventGroup;

xEventGroup = xEventGroupCreate();
if( xEventGroup == NULL ) while(1);

//wait
EventBits_t uxBits;
                                                  // clear, waitAll
uxBits = xEventGroupWaitBits(xEventGroup, , pdTRUE, pdFALSE, portMAX_DELAY); // *    (events)

if(uxBits & _EV){

}

//set
xEventGroupSetBits(xEventGroup, );   // *    (events)

BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xEventGroupSetBitsFromISR(xEventGroup, , &xHigherPriorityTaskWoken);   // *    (events)
portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

//other interesting functions
    pdMS_TO_TICKS( xTimeInMS ); //(xTimeInS / configTICK_RATE_HZ)

    configASSERT( xEventGroup );
    xEventGroupSync( xEventGroup, uxBitsToSet, uxBitsToWaitFor, xTicksToWait); // * * * *
    xEventGroupGetBits( xEventGroup ); // *
    xEventGroupClearBits( xEventGroup, _EV); // * *

/****************/
/** SEMAPHORES **/
/****************/

static SemaphoreHandle_t xSemaphore_;

xSemaphore_ = xSemaphoreCreateCounting( xMax, xIni ); // * *   /// COUNTING ///
if( xSemaphore_ == NULL ) while(1);

xSemaphore_ = xSemaphoreCreateBinary();  /// BINARY ///
if( xSemaphore_ == NULL ) while(1);

xSemaphoreTake(xSemaphore_, portMAX_DELAY); // *

xSemaphoreGive( xSemaphore_ ); // *

BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR( xSemaphore_, &xHigherPriorityTaskWoken ); // *
portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


/************/
/** QUEUES **/
/************/

#define _QUEUE_SIZE 10 // * *

static QueueHandle_t xQueue_;

xQueue_ = xQueueCreate( _QUEUE_SIZE, sizeof(elem_type) ); // * * *
if( xQueue_ == NULL ) while(1); // *

elem_type pvItemRxQueue;
xQueueReceive( xQueue_, &pvItemRxQueue, portMAX_DELAY); // * * *

elem_type pvItemRxQueue;
xQueueSend( xQueue_, ( void * ) &pvItemRxQueue, ( TickType_t ) xTicksToWait ); // * * *

BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xQueueSendFromISR( xQueue_, ( void * ) &pvItemTxQueue, &xHigherPriorityTaskWoken ); // * *
portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

xQueueReset(xQueue_); // *

//other
xQueuePeek(xQueue_, ( void * ) &pvItemRxQueue, xTicksToWait); // * * 
xQueuePeekFromISR(xQueue_, ( void * ) &pvItemRxQueue); // * * 

 xQueueOverwrite( xQueue_, &pvItemToQueue); // * *
 xQueueOverwriteFromISR( xQueue_, &pvItemToQueue, &xHigherPriorityTaskWoken); // * *

//queue example
xQueueSend( xQueue_, ( void * ) &pvItemRxQueue, ( TickType_t ) xTicksToWait ); // * * *
xEventGroupSetBits(xEventGroup, _QUEUE_IS_DATA_AVAILABLE);   // * *

xQueueSendFromISR( xQueue_, ( void * ) &pvItemTxQueue, &xHigherPriorityTaskWoken ); // * *
xEventGroupSetBits(xEventGroup, _QUEUE_IS_DATA_AVAILABLE);   // * *

uxBits = xEventGroupWaitBits(xEventGroup, _QUEUE_DATA_AVAILABLE, pdFALSE, pdFALSE, portMAX_DELAY); // *  *

if(uxBits & _QUEUE_DATA_AVAILABLE){	// *
	elem_type pvItemRxQueue;
	xQueueReceive( xQueue_, &pvItemRxQueue, 0); // * *

	Tx_data

	if(uxQueueSpacesAvailable(xQueue_) == _QUEUE_SIZE)				 // * *
		xEventGroupClearBits( xEventGroup, _QUEUE_IS_DATA_AVAILABLE); // * 
}


/*****************/
/** QUEUES SETS **/
/*****************/

#define configUSE_QUEUE_SETS                1

#define QUEUES_LENGTH xSize // *
#define COUNTING_SEMAPHORES_LENGTH xMax // *
#define BINARY_SEMAPHORES_LENGTH (1) // *
#define _SET_SIZE ( QUEUES_LENGTH + COUNTING_SEMAPHORES_LENGTH + COUNTING_SEMAPHORES_LENGTH) // *

// Init QueueSet
QueueSetHandle_t xQueueSet_ = xQueueCreateSet( _SET_SIZE ); // *
if( xQueueSet_ == NULL ) while(1); // *
xQueueAddToSet( xQueue_, xQueueSet_ ); // * *
xQueueAddToSet( xSemaphore_, xQueueSet_ ); // * *
xQueueAddToSet( xSemaphore_, xQueueSet_ ); // * *


//wait
for(;;){
	static QueueSetMemberHandle_t xActivatedMember;
    xActivatedMember = xQueueSelectFromSet( xQueueSet_, portMAX_DELAY);

    if( xActivatedMember == xQueue_ ){ // *
        RxItemType xReceivedFromQueue; // *
        xQueueReceive( xActivatedMember, &xReceivedFromQueue, 0 );
        
    }
    else if( xActivatedMember == xSemaphore_ ){
        xSemaphoreTake( xActivatedMember, 0 );
        
    }
    else if( xActivatedMember == xSemaphore_ ){
        xSemaphoreTake( xActivatedMember, 0 );
       
    }
    else{
        /* time expired. */
    }
}


/************/
/** NOTIFY **/
/************/

static TaskHandle_t xTask_;

ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

xTaskNotifyGive( xTask_ ); // *

BaseType_t xHigherPriorityTaskWoken = pdFALSE;
vTaskNotifyGiveFromISR( xTask_, &xHigherPriorityTaskWoken ); // *
portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


/*********************/
/** SOFTWARE TIMERS **/
/*********************/

portMAX_DELAY, 0,  pdMS_TO_TICKS(per_ms), per_sec*configTICK_RATE_HZ, configTICK_RATE_HZ/frec_hz

void vTimerCallback_( TimerHandle_t xTimer ){
	uint32_t ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

	if (ulCount == 0)
		xTimerDelete( xTimer, 0);//xTimerStop( xTimer, 0 );
	else
		vTimerSetTimerID( xTimer, ( void * ) ulCount );

	ulCount--;
}

//											                 AutoReload, ID init
TimerHandle_t xTimer_ = xTimerCreate( "Timer", xPeriodTicks, pdTRUE, ( void * ) 0, vTimerCallback_ ); // * * *
if(xTimer_ == NULL) while(1); // *

xTimerStart( xTimer_, portMAX_DELAY ); // * 
xTimerStop( xTimer_, portMAX_DELAY ); // * 
xTimerChangePeriod( xTimer_, xNewPeriod, portMAX_DELAY ); // * * 
xTimerReset( xTimer_, portMAX_DELAY ); // * 
pvTimerGetTimerID( xTimer ); // * 
vTimerSetTimerID ( xTimer, ( void * ) ulCount ); // * *

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ISR, INTR, RGB, PWM, BUTTONS ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*********/
/** RGB **/
/*********/

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


/******************/
/** PULL UP/DOWN **/
/******************/

//															   (maxCurrent 12 mA)         (mode)
MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);


/******************/
/** BUTTONS INTR **/
/******************/

// buttons intterrupt (GPIO_PIN_4, GPIO_PIN_0) 
void configButtonsINTR(){    
    // Config Buttons                                            
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_GPIOF );  
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_GPIOF );   
    ButtonsInit();
    GPIOIntClear(   GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);  
    GPIOIntTypeSet( GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);  // *  
    GPIOIntEnable(  GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);    
        IntRegister(    INT_GPIOF, Buttons_ISR );   
    	IntPrioritySet( INT_GPIOF, configMAX_SYSCALL_INTERRUPT_PRIORITY ); 
        IntEnable(      INT_GPIOF );                
}

// simple buttons ISR
#define DBOUNCE         10

void Buttons_ISR(void){
    static int last_left = -DBOUNCE, last_right = -DBOUNCE;
    int time = xTaskGetTickCountFromISR();
    int intStatus = GPIOIntStatus(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    int pinStatus = GPIOPinRead(  GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

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

// buttons ISR (long short detection)
#define DBOUNCE     10
#define LONG_PRESS  pdMS_TO_TICKS(1000)

void Buttons_ISR(void){
    static int last_left = -DBOUNCE, last_right = -DBOUNCE, press_left, press_right;
    int time = xTaskGetTickCountFromISR();
    int intStatus = GPIOIntStatus(  GPIO_PORTF_BASE, true);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(intStatus & LEFT_BUTTON)
        if ( (time-last_left) > DBOUNCE ){
            last_left = time;

            if( GPIOIntTypeGet( GPIO_PORTF_BASE, 4 ) == GPIO_RISING_EDGE ){ //release
                if( (time-press_left) > LONG_PRESS ){ //long press
                   
                }
                else{ // short press
                    
                }
                GPIOIntTypeSet( GPIO_PORTF_BASE, LEFT_BUTTON, GPIO_FALLING_EDGE);  
            }
            else{ //pressed
                press_left = time;
                GPIOIntTypeSet( GPIO_PORTF_BASE, LEFT_BUTTON, GPIO_RISING_EDGE);  
            }
        }

    if(intStatus & RIGHT_BUTTON)
        if ( (time-last_right) > DBOUNCE ){
            last_right = time;

            if( GPIOIntTypeGet( GPIO_PORTF_BASE, 0 ) == GPIO_RISING_EDGE ){ //release
                if( (time-press_right) > LONG_PRESS ){ //long press
                   
                }
                else{ // short press
                    
                }
                GPIOIntTypeSet( GPIO_PORTF_BASE, RIGHT_BUTTON, GPIO_FALLING_EDGE);  // *
            }
            else{ //pressed
                press_right = time;
                GPIOIntTypeSet( GPIO_PORTF_BASE, RIGHT_BUTTON, GPIO_RISING_EDGE);  // *
            }
        }

    GPIOIntClear( GPIO_PORTF_BASE, intStatus);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// interrupt
void configInINTR(){
    // Config Interrupt                                   //Check
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_GPIOl );   // *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_GPIOl );   // *
    GPIOPinTypeGPIOInput( GPIO_PORTl_BASE, GPIO_PIN_n );  // * *
    GPIOIntClear(         GPIO_PORTl_BASE, GPIO_PIN_n );  // * *
    GPIOIntTypeSet(       GPIO_PORTl_BASE, GPIO_PIN_n, GPIO_FALLLING_EDGE);  // * * * 
    GPIOIntEnable(        GPIO_PORTl_BASE, GPIO_PIN_n );  // * *
        IntRegister(    INT_GPIOl, voidGPIOl_ISR );       // *
        IntPrioritySet( INT_GPIOl, configMAX_SYSCALL_INTERRUPT_PRICORITY ); // *
        IntEnable(      INT_GPIOl );                      // *
}

/*********/
/** PWM **/
/*********/

//(TIMER0B)T0CCP1  --> PF1 (LED rojo)
//(TIMER1A)T1CCP0  --> PF2 (LED azul)
//(TIMER1B)T1CCP1  --> PF3 (LED verde)

#define SYSCTL_PERIPH_GPIOl     SYSCTL_PERIPH_GPIOF
#define GPIO_PORTl_BASE         GPIO_PORTF_BASE
#define GPIO_PIN_n              GPIO_PIN_1
#define SYSCTL_PERIPH_TIMERn    SYSCTL_PERIPH_TIMER0
#define TIMERn_BASE             TIMER0_BASE
#define TIMER_CFG_l_PWM         TIMER_CFG_B_PWM
#define TIMER_l                 TIMER_B

void configPWM(){
	// config PWM
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_GPIOl);      // *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_GPIOl);      // *
    GPIOPinTypeGPIOOutput( GPIO_PORTl_BASE, GPIO_PIN_n);    // * *
    GPIOPinWrite(          GPIO_PORTl_BASE, GPIO_PIN_n, 0); // *
    GPIOPinConfigure(GPIO_PF1_TnCCPn);                      // **
    GPIOPinTypePWM(GPIO_PORTl_BASE, GPIO_PIN_n);            // * *

    SysCtlPeripheralEnable(      SYSCTL_PERIPH_TIMERn);     // * *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_TIMERn);     // * *
    TimerConfigure( TIMERn_BASE, TIMER_CFG_l_PWM | TIMER_CFG_SPLIT_PAIR);   // * *
    TimerLoadSet(   TIMERn_BASE, TIMER_l, ui32Period);    // * *
    TimerMatchSet(  TIMERn_BASE, TIMER_l, ui32DutyCycle); // * *
    TimerEnable(    TIMERn_BASE, TIMER_l);  // * *

}

//de un examen
	/*	WTIMER0A y WTIMER0B PC4 y PC5 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinConfigure(GPIO_PC4_WT0CCP0);
	GPIOPinConfigure(GPIO_PC5_WT0CCP1);
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);

	// Habilita el WTimer0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER0);
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
	TimerLoadSet(WTIMER0_BASE, TIMER_BOTH,  PERIOD_20ms);
	TimerMatchSet(WTIMER0_BASE, TIMER_BOTH, CYCLE_1_5ms_STOP);
	TimerEnable(WTIMER0_BASE, TIMER_BOTH);


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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TIMERS /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*****************/
/** BOTH TIMERS **/
/*****************/

void configBothTimer(){
    uint32_t ui32Period = (SysCtlClockGet() / 10) / 2; // SysCtlClockGet() -> 1seg -> 1s/10/2 = 0.1/2 = 0.05s => 20Hz

    // Config TIMERn                                      //check
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_TIMERn );  // *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_TIMERn );  // *
    TimerConfigure( TIMERn_BASE, TIMER_CFG_PERIODIC);     // *
    //TimerLoadSet( TIMERn_BASE, TIMER_A, ui32Period -1); // * *
        IntRegister(    INT_TIMERnA, voidTIMERnA_ISR);    // * * 
    	IntPrioritySet( INT_TIMERnA, configMAX_SYSCALL_INTERRUPT_PRIORITY ); // *
        IntEnable(      INT_TIMERnA );                    // *
    TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
    TimerIntEnable( TIMERn_BASE, TIMER_TIMA_TIMEOUT );    // * 
    TimerDisable(   TIMERn_BASE, TIMER_A );               // * 
}

void voidTIMERnA_ISR(void){								// *
	TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT );  // *
}

TimerDisable(  TIMERn_BASE, TIMER_A );				 // * 
TimerLoadSet(  TIMERn_BASE, TIMER_A, ui64Period -1); // * *
TimerIntClear( TIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(   TIMERn_BASE, TIMER_A );				 // * 


/******************/
/** SPLIT TIMERS **/
/******************/

void configSplitTimer(){
    uint16_t ui32PeriodA = ui32PeriodB = (SysCtlClockGet() / 10) / 2; // SysCtlClockGet() -> 1seg -> 1s/10/2 = 0.1/2 = 0.05s => 20Hz

    // Config TIMERn                                        //check
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_TIMERn );    // *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_TIMERn );    // *
    TimerConfigure( TIMERn_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC ); // *
    //TimerLoadSet( TIMERn_BASE, TIMER_A, ui32PeriodA -1);  // * *
    //TimerLoadSet( TIMERn_BASE, TIMER_B, ui32PeriodB -1);  // * *
        IntRegister(    INT_TIMERnA, voidTIMERnA_ISR );     // * *
        IntRegister(    INT_TIMERnB, voidTIMERnB_ISR );     // * *
    	IntPrioritySet( INT_TIMERnA, configMAX_SYSCALL_INTERRUPT_PRIORITY ); // *
    	IntPrioritySet( INT_TIMERnB, configMAX_SYSCALL_INTERRUPT_PRIORITY ); // *
        IntEnable(      INT_TIMERnA );                      // *
        IntEnable(	    INT_TIMERnB );                      // *
    TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT ); // *
    TimerIntEnable( TIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT ); // *
    TimerDisable(  TIMERn_BASE, TIMER_A | TIMER_B );          // *
}

void voidTIMERnA_ISR(void){								// *
	TimerIntClear(  TIMERn_BASE, TIMER_TIMA_TIMEOUT );  // *
}

void voidTIMERnB_ISR(void){								// *
	TimerIntClear(  TIMERn_BASE, TIMER_TIMB_TIMEOUT );  // *
}

TimerDisable(  TIMERn_BASE, TIMER_A );				 // * 
TimerLoadSet(  TIMERn_BASE, TIMER_A, ui32Period -1); // * *
TimerIntClear( TIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(   TIMERn_BASE, TIMER_A );				 // * 
TimerDisable(  TIMERn_BASE, TIMER_B );				 // * 
TimerLoadSet(  TIMERn_BASE, TIMER_B, ui32Period -1); // * *
TimerIntClear( TIMERn_BASE, TIMER_TIMB_TIMEOUT );    // *
TimerEnable(   TIMERn_BASE, TIMER_B );				 // * 

/**********************/
/** BOTH WIDE TIMERS **/
/**********************/

void configWideBothTimer(){
    uint64_t ui64Period = (SysCtlClockGet() * 1000);

    // Config WTIMERn                                      //check
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_WTIMERn );  // *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_WTIMERn );  // *
    TimerConfigure(   WTIMERn_BASE, TIMER_CFG_PERIODIC );  // *
    //TimerLoadSet64( WTIMERn_BASE, ui64Period -1);        // * *
        IntRegister(    INT_WTIMERnA, voidWTIMERnA_ISR );  // * *
    	IntPrioritySet( INT_WTIMERnA, configMAX_SYSCALL_INTERRUPT_PRIORITY ); // *
        IntEnable(      INT_WTIMERnA );                    // *
    TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
    TimerIntEnable( WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // * 
    TimerDisable(   WTIMERn_BASE, TIMER_A );               // * 
}

void voidWTIMERnA_ISR(void){							 // *
	TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT );  // *	
}

TimerDisable(   WTIMERn_BASE, TIMER_A );		     // * 
TimerLoadSet64( WTIMERn_BASE, ui64Period -1); 		 // * * 
TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT );  // *
TimerEnable(    WTIMERn_BASE, TIMER_A );			 // * 


/***********************/
/** SPLIT WIDE TIMERS **/
/***********************/

void configSplitTimer(){
    uint16_t ui32PeriodA = ui32PeriodB = (SysCtlClockGet() / 10) / 2; // SysCtlClockGet() -> 1seg -> 1s/10/2 = 0.1/2 = 0.05s => 20Hz

    // Config WTIMERn                                       //check
    SysCtlPeripheralEnable(      SYSCTL_PERIPH_WTIMERn );   // *
    SysCtlPeripheralSleepEnable( SYSCTL_PERIPH_WTIMERn );   // *
    TimerConfigure( WTIMERn_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC ); // *
    //TimerLoadSet( WTIMERn_BASE, TIMER_A, ui32PeriodA -1);   // * *
    //TimerLoadSet( WTIMERn_BASE, TIMER_B, ui32PeriodB -1);   // * *
        IntRegister(    INT_WTIMERnA, voidWTIMERnA_ISR );   // * *
        IntRegister(    INT_WTIMERnB, voidWTIMERnB_ISR );   // * *
    	IntPrioritySet( INT_WTIMERnA, configMAX_SYSCALL_INTERRUPT_PRIORITY ); // *
    	IntPrioritySet( INT_WTIMERnB, configMAX_SYSCALL_INTERRUPT_PRIORITY ); // *
        IntEnable(      INT_WTIMERnA );                     // *
        IntEnable(	    INT_WTIMERnB );                     // *
    TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT ); // *
    TimerIntEnable( WTIMERn_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT ); // *
    TimerDisable(   WTIMERn_BASE, TIMER_A | TIMER_B );      // *
}

void voidWTIMERnA_ISR(void){						    // *
	TimerIntClear(  WTIMERn_BASE, TIMER_TIMA_TIMEOUT ); // *
}

void voidWTIMERnB_ISR(void){							// *
	TimerIntClear(  WTIMERn_BASE, TIMER_TIMB_TIMEOUT ); // *
}

TimerDisable(  WTIMERn_BASE, TIMER_A );		          // * 
TimerLoadSet(  WTIMERn_BASE, TIMER_A, ui32Period -1); // * * 
TimerIntClear( WTIMERn_BASE, TIMER_TIMA_TIMEOUT );    // *
TimerEnable(   WTIMERn_BASE, TIMER_A );			      // * 
TimerDisable(  WTIMERn_BASE, TIMER_B );		          // * 
TimerLoadSet(  WTIMERn_BASE, TIMER_B, ui32Period -1); // * * 
TimerIntClear( WTIMERn_BASE, TIMER_TIMB_TIMEOUT );    // *
TimerEnable(   WTIMERn_BASE, TIMER_B );			      // * 


//Other interesting functions

TimerDisable();
TimerLoadGet();
TimerLoadGet64();
TimerControlTrigger();


/*********************/
/** MESSAGE RECEIVE **/
/*********************/

        case MESSAGE_BOTON_MICECATS:{
            MESSAGE_BOTON_MICECATS_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0){
                
            }
            else
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
         
            break;
        }


/*********************/
/** FILE PROTOTYPES **/
/*********************/

//fichero.h----------------------------------------

/// Autor: Ramon Heras Garcia SE ///

#ifndef FILE_H
#define FILE_H



#endif /* fin */

//fichero.c----------------------------------------

/// Autor: Ramon Heras Garcia SE ///

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "drivers/configADC.h"

#include <remotelink.h>
#include <serialprotocol.h>

//-----------------------------------------------


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// SRQUENCER, ADC, TRIGGER, ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void configADC_DisparaADC(void)
{
	ADCProcessorTrigger(ADC_BASE,ADC_SSC);
}


void configADC_SetGPIOTrigger(uint8_t ui8Pins){
    stream_mode = false;

    
    //Disable all
    TimerDisable(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL);
    TimerControlTrigger(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL, false);
    GPIOADCTriggerDisable(GPIO_PORTF_BASE, 0xFF);

    //Configure GPIO
    GPIOADCTriggerEnable(GPIO_PORTF_BASE, ui8Pins);

    
    //Configure Sequencer
    ADCSequenceEnable(ADC_BASE,ADC_SSC);
        ADCSequenceConfigure(ADC_BASE,ADC_SSC,ADC_TRIGGER_EXTERNAL,0);
    ADCSequenceEnable(ADC_BASE,ADC_SSC);
}

void configADC_SetTimerTrigger(float frec, bool status){
    stream_mode = true;

    //Disable all
    TimerDisable(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL);
    TimerControlTrigger(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL, false);
    GPIOADCTriggerDisable(GPIO_PORTF_BASE, 0xFF);

    
    //Configure Timer
    TimerConfigure(TRIGGER_TIMER_BASE, TIMER_CFG_PERIODIC);
    uint32_t per = SysCtlClockGet()/frec;
    TimerLoadSet(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL, per-1);
    TimerControlTrigger(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL, true);

    if(status)
        TimerEnable(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL);
    else
        TimerDisable(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL);

    //Configure Sequencer
    ADCSequenceDisable(ADC_BASE,ADC_SSC);
        ADCSequenceConfigure(ADC_BASE,ADC_SSC,ADC_TRIGGER_TIMER,0);
    ADCSequenceEnable(ADC_BASE,ADC_SSC);
}

void configADC_SetGUITrigger(void){
    stream_mode = false;

    
    //Disable all
    TimerDisable(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL);
    TimerControlTrigger(TRIGGER_TIMER_BASE, TRIGGER_TIMER_TIMER_SEL, false);
    GPIOADCTriggerDisable(GPIO_PORTF_BASE, 0xFF);

    
    //Configure and Trigger Sequencer
    ADCSequenceDisable(ADC_BASE,ADC_SSC);
        ADCSequenceConfigure(ADC_BASE,ADC_SSC,ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceEnable(ADC_BASE, ADC_SSC);
}


void configADC_IniciaADC(void)
{
                //Enable ADC
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
			    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

			    //Analog INput Port option A
				SysCtlPeripheralEnable(AIN_SYSCTL_PERIPH_A);
				SysCtlPeripheralSleepEnable(AIN_SYSCTL_PERIPH_A);
				GPIOPinTypeADC(AIN_GPIO_PORT_BASE_A, AIN_GPIO_PIN_A);

				//Analog INput Port option B
				SysCtlPeripheralEnable(AIN_SYSCTL_PERIPH_B);
				SysCtlPeripheralSleepEnable(AIN_SYSCTL_PERIPH_B);
				GPIOPinTypeADC(AIN_GPIO_PORT_BASE_B, AIN_GPIO_PIN_B);

			    //Timer config (ADC sampling)
			    SysCtlPeripheralEnable(TRIGGER_SYSCTL_PERIPH_TIMER);
			    SysCtlPeripheralSleepEnable(TRIGGER_SYSCTL_PERIPH_TIMER);
			    TimerConfigure(TRIGGER_TIMER_BASE, TIMER_CFG_PERIODIC);

				//CONFIGURAR SECUENCIADOR 1 (DEFAULT)
                ADCClockConfigSet(ADC_BASE, ADC_CLOCK_RATE_FULL, 1);               //Configuramos la velocidad de conversion al maximo (1MS/s)
                ADCSequenceConfigure(ADC_BASE,ADC_SSC,ADC_TRIGGER_PROCESSOR,0);	//Disparo software (processor trigger)

                ADCSequenceStepConfigure(ADC_BASE,ADC_SSC,0,ADC_CTL_CH0);
                ADCSequenceStepConfigure(ADC_BASE,ADC_SSC,1,ADC_CTL_CH1);
                ADCSequenceStepConfigure(ADC_BASE,ADC_SSC,2,ADC_CTL_CH2);
                ADCSequenceStepConfigure(ADC_BASE,ADC_SSC,3,ADC_CTL_CH3);
                ADCSequenceStepConfigure(ADC_BASE,ADC_SSC,4,ADC_CTL_CH4);
                ADCSequenceStepConfigure(ADC_BASE,ADC_SSC,5,ADC_CTL_CH5 |ADC_CTL_IE |ADC_CTL_END );	//La ultima muestra provoca la interrupcion
				ADCSequenceEnable(ADC_BASE,ADC_SSC);

				//Habilita las interrupciones
				ADCIntClear(ADC_BASE,ADC_SSC);
				ADCIntEnable(ADC_BASE,ADC_SSC);
				IntRegister(INT_ADCSSC, configADC_ISR);
				IntPrioritySet(INT_ADCSSC,configMAX_SYSCALL_INTERRUPT_PRIORITY);
				IntEnable(INT_ADCSSC);


				//Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
				cola_adc=xQueueCreate(8,sizeof(ADCsamples_t));
				if (cola_adc==NULL)
				{
					while(1);
				}
}


void configADC_LeeADC(ADCsamples_t *datos){
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void configADC_ISR(void){
	static ADCsamples_t sample;
	portBASE_TYPE higherPriorityTaskWoken=pdFALSE;
	uint32_t aux[N_ANALOG_IN];

    ADCIntClear(ADC_BASE,ADC_SSC);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ADCSequenceDataGet(ADC_BASE,ADC_SSC,(uint32_t *)&aux);//COGEMOS LOS DATOS GUARDADOS

    int i;
    for(i=0; i<N_ANALOG_IN; i++)
        sample[i] = aux[i];

    xQueueSendFromISR(cola_adc,&sample,&higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}


//macros
#define SET(flags, bit)     (flags |=  (0x01 << bit))
#define CLEAR(flags, bit)   (flags &= ~(0x01 << bit))
#define TOGGLE(flags, bit)  (flags ^=  (0x01 << bit))
#define IS_SET(flags, bit)  ((flags >> bit) & 0x01)
#define BIT(pos)            (0x01 << pos)

//definitions
#define ADC_BASE ADC0_BASE
#define ADC_SSC 0
#define INT_ADCSSC INT_ADC0SS0
#define N_ANALOG_IN 6
#define NUM_CHANNELS 4

// Analog inputs (6 pin max)
#define AIN_SYSCTL_PERIPH_A             (SYSCTL_PERIPH_GPIOE)
#define AIN_GPIO_PORT_BASE_A            (GPIO_PORTE_BASE)
#define AIN_GPIO_PIN_A                  (GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0)
#define AIN_SYSCTL_PERIPH_B             (SYSCTL_PERIPH_GPIOD)
#define AIN_GPIO_PORT_BASE_B            (GPIO_PORTD_BASE)
#define AIN_GPIO_PIN_B                  (GPIO_PIN_3|GPIO_PIN_2)

// Trigger timer
#define TRIGGER_SYSCTL_PERIPH_TIMER     (SYSCTL_PERIPH_TIMER3)
#define TRIGGER_TIMER_BASE              (TIMER3_BASE)                    // TIMERi_BASE
#define TRIGGER_TIMER_TIMER_SEL         (TIMER_A)                        // TIMER_A, TIMER_B, TIMER_BOTH

/************************/
/** I2C FOLLADITA 2019 **/
/************************/


//I2C Init 
void InitFolladita2019{
    
    if(I2C_IF_Open(I2C_MASTER_MODE_STD) < 0)
        while (1);

	vTaskDelay(10);

	InitSensor();
}


// poll sensor
#define WHO_AM_I_REGISTER
#define WHO_AM_I_RESPONSE

bool IsConnected(){
    return (WHO_AM_I_RESPONSE==readRegister(WHO_AM_I_REGISTER));
}

#define MAX_ID
#define CHIP_ID

uint16_t getChipID()
{
    return(readRegister16(CHIP_ID) < MAX_ID);
}

//Reads two bytes 
uint16_t readRegister16(uint8_t addr){
    uint16_t resultado;

    if(I2C_IF_ReadFrom(RFD77402_ADDR, &addr, sizeof(uint8_t),(uint8_t *)&resultado, sizeof(uint16_t)) != 0){
        DBG_PRINT("I2C readfrom failed\n\r");
        return (0xFFFF); //Esto es lo que hace la biblioteca original....
    }

    return resultado;
}

//Reads from a given location
uint8_t readRegister(uint8_t addr){
    uint8_t resultado;

    if(I2C_IF_ReadFrom(RFD77402_ADDR, &addr, sizeof(uint8_t), &resultado, sizeof(uint8_t)) != 0){
        DBG_PRINT("I2C readfrom failed\n\r");
        return (0xFF); //Esto es lo que hace la biblioteca original....
    }

    return resultado;
}

//Write a 16 bit value to a spot 
void writeRegister16(uint8_t addr, uint16_t val){
    uint8_t ucData[3] = {addr, (val & 0xFF), (val >> 8)}; //ucData[0] = addr; ucData[1] = (val & 0xFF);ucData[2] = (val >> 8);

    if (I2C_IF_Write(RFD77402_ADDR,ucData,sizeof(ucData),1) != 0){
        DBG_PRINT("I2C write failed\n\r");
    }
}

//Write a value to a spot 
void writeRegister(uint8_t addr, uint8_t val){
    uint8_t ucData[2] = {addr, val}; //ucData[0] = addr; ucData[1] = val;

    if (I2C_IF_Write(ADDR, ucData, sizeof(ucData), 1) != 0) {
        DBG_PRINT("I2C write failed\n\r");
    }

}
