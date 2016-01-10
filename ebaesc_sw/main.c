/*******************************************************************************
*
* File Name: main.c
*
* Description: Main application file generated automatically from the 
*              DSP56800E_Quick_Start stationery
*
* Target: MC56F84789 device
*
*******************************************************************************/


#include "allincludes.h"



/* local prototypes */


/*
 * The main function is invoked from the startup code after 
 * key device resources (PLL, memory) are initialised.
 *
 */

void main (void) 
{
	// Some temporary variables
	
	UInt8 ui8Temp0 = 0;
	UInt8 ui8Temp1 = 0;
	
	/* initialise SYS module */
    ioctl(SYS, SYS_INIT, NULL);

    /* configure COP module */
    ioctl(COP, COP_INIT, NULL);
    
    /* configure all GPIO modules */
    ioctl(GPIO, GPIO_INIT_ALL, NULL);
    ioctl(GPIO_A, GPIO_INIT, NULL);
    ioctl(GPIO_B, GPIO_INIT, NULL);
    ioctl(GPIO_C, GPIO_INIT, NULL);
    ioctl(GPIO_D, GPIO_INIT, NULL);
    ioctl(GPIO_E, GPIO_INIT, NULL);
    ioctl(GPIO_F, GPIO_INIT, NULL);

    /* 
     * TODO: put your other module initialisation calls here
     *   e.g. ioctl(SCI, SCI_INIT, NULL)
     *
     */
    ioctl(XBAR_A, XBAR_A_INIT, null);
    ioctl(EFPWMA, EFPWM_INIT, null);
    ioctl(EFPWM_A_SUB0, EFPWM_INIT, null);
    ioctl(EFPWM_A_SUB1, EFPWM_INIT, null);
    ioctl(EFPWM_A_SUB2, EFPWM_INIT, null);
    
    ioctl(ADC_1, ADC_INIT, null);
    ioctl(PIT_0, PIT_INIT, NULL);
    ioctl(SPI_0, SPI_INIT, NULL);
    ioctl(SCI_0, SCI_INIT, NULL);
    ioctl(SCI_1, SCI_INIT, NULL);
    ioctl(QTIMER_B0, QT_INIT, NULL);
    ioctl(QTIMER_B1, QT_INIT, NULL);
    ioctl(QTIMER_B2, QT_INIT, NULL);
    ioctl(QTIMER_B3, QT_INIT, NULL);
    ioctl(FCAN, FCAN_INIT, NULL);
    
    FMSTR_Init();    
    
    // Init SCI0 ring buffer
    RB_Init(&SCI0RXBuff, SCI0RXBuffer, 128);
    
    /* initialise interrupt controller and enable interrupts */
    ioctl(INTC, INTC_INIT, NULL);
    archEnableInt();

    /*
     * TODO: perform additional initialisations here 
     *
     */
    
    // RS485 communication setup
    RS485_initData();
    
    // Set all flag values to initial value
    SYS_DEBUG_MODE = 1;
    SYSTEM_CALIBRATED = 0;
    SYSTEM_RUN_SENSORED = 0;
    CONTROL_TORQUE = 0;
    CONTROL_SPEED = 0;
    SENSORLESS_BEMF_ON = 0;
    
    // Enable gate driver
    EN_GATE_ON;
    delay(30000);
    
    // Initialize system variables
    InitSysVars(1);
    
    // Calculate float values from parameters 
    calculateFloats();
    
    // Set current offsets
    SYS_ZERO_CURRENT = 1;
    
    
    // Initialize MOSFET driver
	if(0 != InitDRV8301(0,31,1))
	{
		// Driver did not initialize properly
		DRV8301_CONFIGURED = 0;
		SYSTEM.systemState = SYSTEM_FAULT;
	}
	else
	{
		// After everything is initialised, DRV8301_CONFIGURED = 1 moves system state from boot to init.
		DRV8301_CONFIGURED = 1;
	}
    while(1)
    {
    	// Check freemaster
    	FMSTR_Poll();
    	
    	// Check RS485 comm
    	if(0 != SCI0RXBuff.count)
    	{
			// Call RS485 state machine
			RS485_States_slave(RB_pop(&SCI0RXBuff));    		
    	}

    	// Check test bit - for testing code
    	if(SYSTEM_TEST_BIT)
    	{
    		SYSTEM_TEST_BIT = 0;		
    	}
    	
    	// Recalculate factors?
    	if(SYSTEM_RECALCULATE_FACTORS)
    	{
    		SYSTEM_RECALCULATE_FACTORS = 0;
    		calculateFactors();
    	}    	

         /* feed the watchdog periodically */
         ioctl(COP, COP_CLEAR_COUNTER, NULL);

    }
}
