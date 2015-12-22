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
    
    FMSTR_Init();    
    

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
		DRV8301_CONFIGURED = 1;
	}

    while(1)
    {
        /*
         * TODO: put your main-loop code here 
         *
         */
    	// Check freemaster
    	FMSTR_Poll();
    	
    	/*
    	// Send out ping
    	if(ui8Temp1 != RS485Address)
    	{
    		ui8Temp1 = RS485Address;
			// Test serial comm - request
			// Setup data
			// FF FF 01 04 02 1E 04 D6
			ui8SerialBuffer[0] = 0xff;
			ui8SerialBuffer[1] = 0xff;
			// ID
			ui8SerialBuffer[2] = RS485Address;
			ui8SerialBuffer[3] = 0x02;
			ui8SerialBuffer[4] = 0x01;
			ui8SerialBuffer[5] = ~(RS485Address + 0x03);
			// Enable transmitter
			RS485_ENABLE_TX;
			// Send
			for(ui8Temp0 = 0; ui8Temp0 < 6; ui8Temp0++)
			{
				while(!RS485_TEST_TX_EMPTY)
				{
					asm(nop);
				}
				RS485_WRITE(ui8SerialBuffer[ui8Temp0]);
			}
			// Wait for end of transmission
			while(!RS485_TEST_TX_IDLE)
			{
				asm(nop);
			}
			// Enable receiver
			RS485_ENABLE_RX;   
    	}
    	*/
    	// Check test bit - for testing code
    	if(SYSTEM_TEST_BIT)
    	{
    		SYSTEM_TEST_BIT = 0;
    		
    		if(RS485Address < 254)
    		{
				// Test serial comm - request
				// Setup data
				// FF FF 01 04 02 1E 04 D6
				ui8SerialBuffer[0] = 0xff;
				ui8SerialBuffer[1] = 0xff;
				// ID
				ui8SerialBuffer[2] = RS485Address;
				ui8SerialBuffer[3] = 0x02;
				ui8SerialBuffer[4] = 0x01;
				ui8SerialBuffer[5] = ~(RS485Address + 0x03);
				// Enable transmitter
				RS485_ENABLE_TX;
				// Send
				for(ui8Temp0 = 0; ui8Temp0 < 6; ui8Temp0++)
				{
					while(!RS485_TEST_TX_EMPTY)
					{
						asm(nop);
					}
					RS485_WRITE(ui8SerialBuffer[ui8Temp0]);
				}
				// Wait for end of transmission
				while(!RS485_TEST_TX_IDLE)
				{
					asm(nop);
				}
				// Enable receiver
				RS485_ENABLE_RX;      			
    			
    			
    			
    			RS485Address++;
    		}
    		

 		
    		// Clear buffer
    		for(ui8Temp0 = 0; ui8Temp0 < 8; ui8Temp0++)
    		{
    			ui8SerialBuffer[ui8Temp0] = 0;
    		}
    		// Wait for data
    		/*
    		for(ui8Temp0 = 0; ui8Temp0 < 8; ui8Temp0++)
    		{
    			while(!RS485_TEST_RX_FULL)
    			{
    				asm(nop);
    			}
    			ui8SerialBuffer[ui8Temp0] = RS485_READ;
    		}   		
    		*/
    		
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
