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
	UWord32 UW32FlashResult = 0;
	
	UWord8 uw8Test = 0;
	UWord16 uw16Test = 0;
	UWord32 uw32Test = 0;
	
	FCAN_MB *MB;
	int i = 0;
	int code = 0;
	
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
    //ioctl(QTIMER_A1, QT_INIT, NULL);
    ioctl(QTIMER_B0, QT_INIT, NULL);
    ioctl(QTIMER_B1, QT_INIT, NULL);
    ioctl(QTIMER_B2, QT_INIT, NULL);
    ioctl(QTIMER_B3, QT_INIT, NULL);
    //ioctl(HSCMP_A, HSCMP_INIT, NULL);
    ioctl(FCAN, FCAN_INIT, NULL);
    //ioctl(ADC16, ADC16_INIT, NULL);
    
    FMSTR_Init();    
    
    // Initialise SCI0 ring buffer
    RB_Init(&SCI0RXBuff, SCI0RXBuffer, 256);
    
    // Check EEPROM
    UW32FlashResult = GetEepromInfo();
    UW32FlashResult = UW32FlashResult & 0x0000ffff;
    // Check uw16EEESize and uw16EEBackUpFlashSize
    // If flash not partitioned, exec next function
    if(((uw16EEESize&0x00ff) == 0xff) && ((uw16EEBackUpFlashSize&0x00ff) == 0xff) && (UW32FlashResult == EEPROM_FLASHDRV_SUCCESS))
    {
    	UW32FlashResult = DEFlashPartition(EEESIZE_2048B,DEPART_32K);
    }    
    else
    {
    	UW32FlashResult = SetEEEEnable();
    }    
    
    
    
    /* initialise interrupt controller and enable interrupts */
    ioctl(INTC, INTC_INIT, NULL);
    archEnableInt();

    /*
     * TODO: perform additional initialisations here 
     *
     */
    
    // RS485 communication setup
    RS485_initData(&RS485DataStruct);
    
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
    
    
	// Initialise system variables to default values
	InitSysVars(1);      
	/*
    // Check EEPROM sys vars
    EEPROMReadi16(0, &SYSTEM.i16EEPROMStoreDone);
    if(0 == SYSTEM.i16EEPROMStoreDone)
    {
    	// Store to EEPROM
    	StoreEEPROM();
    }
    else
    {
    	// Load from EEPROM
    	LoadEEPROM();
    }
    */

    
    // Calculate float values from parameters 
    calculateFloats();
    
    // Set current offsets
    SYS_ZERO_CURRENT = 1;
    
    // Init CAN buffers
    CAN_Init();
    
    // Gain = 40 (x20 DRV, x2 AD)
    //SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.25);
    SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.5);
    /*
    // Initialise MOSFET driver
	if(0 != InitDRV8301(0,31,1))
	{
		// Driver did not initialise properly
		DRV8301_CONFIGURED = 0;
		SYSTEM.systemState = SYSTEM_FAULT;
	}
	else
	{
		// After everything is initialised, DRV8301_CONFIGURED = 1 moves system state from boot to init.
		DRV8301_CONFIGURED = 1;
	}
	
	ioctl(ADC16, ADC16_WRITE_SC1_REG, 1);
	*/
    while(1)
    {
    	// Check freemaster
    	FMSTR_Poll();
    	
    	// Check RS485 comm
    	/*
    	if(0 != SCI0RXBuff.count)
    	{
			// Call RS485 state machine
			RS485_States_slave(RB_pop(&SCI0RXBuff));    		
    	}
*/
		switch(i8LEDTest)
		{
			case 0:
			{
				break;
			}
			case 1:
			{
				LED_Y_ON;
				i8LEDTest = 0;
				break;
			}			
			case 2:
			{
				LED_Y_OFF;
				i8LEDTest = 0;
				break;
			}	
			case 3:
			{
				LED_R_ON;
				i8LEDTest = 0;
				break;
			}			
			case 4:
			{
				LED_R_OFF;
				i8LEDTest = 0;
				break;
			}
			case 5:
			{
				LED_G_ON;
				i8LEDTest = 0;
				break;
			}			
			case 6:
			{
				LED_G_OFF;
				i8LEDTest = 0;
				break;
			}
			default:
			{
				i8LEDTest = 0;
				break;
			}
		}
				
    	switch(i8ParamTest)
    	{
			case 0:
			{
				break;
			}
			case 1:
			{
				calculateFloats();
				i8ParamTest = 0;
				break;
			}			
			case 2:
			{
				MCAT_Load();
				i8ParamTest = 0;
				break;
			}	
			default:
			{
				i8ParamTest = 0;
				break;
			}
    	}
    	
    	switch(i8EEPROMOp)
    	{
			case 0:
			{
				break;
			}
			case 1:
			{
				StoreEEPROM();
				i8EEPROMOp = 0;
				break;
			}
			case 2:
			{
				SYSTEM.i16MotorID = 0;
				SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain = 0;
				LoadEEPROM();
				i8EEPROMOp = 0;
				break;			
			}
			default:
			{
				i8EEPROMOp = 0;
				break;
			}
    	}
    	
    	switch(i8CANTest)
    	{
    		case 0:    	
			{
				break;
			}
    		case 1:
    		{
    			if(ioctl(FCAN, FCAN_TEST_READY, null))
    			{
    				// Get free MB
    				for(i=0;i<14;i++)
    				{
    					MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
    					// Get code
    					code = ioctl(MB, FCANMB_GET_CODE, null);
    					if(code == 0b1000)
    					{
    						// Write ID
    						//MB->id = 0xff4e2123;
    						// Write message
    						MB->data[0] = 0xaaaa;
    						MB->data[1] = 0x5555;
    						ioctl(MB, FCANMB_SET_ID, 10 | FCAN_ID_EXT);
    						ioctl(MB, FCANMB_SET_LEN, 8);
    						ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXONCE);		
    						i = 15;
    					}
    				}			
    			}
    			i8CANTest = 0;
    			break;
    		}
    		case 2:
    		{
    			if(ioctl(FCAN, FCAN_TEST_READY, null))
    			{
    				MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 0);
    				// Get code
    				code = ioctl(MB, FCANMB_GET_CODE, null);
    				if(0 == code)
    				{
    					ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXVOID);	
    				}
    				else
    				{
    					ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXVOID);
    				}
    			}
    			i8CANTest = 0;
    			break;
    		}
    		case 3:
    		{
    			if(ioctl(FCAN, FCAN_TEST_READY, null))
    			{
    				MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 1);
    				// Get code
    				code = ioctl(MB, FCANMB_GET_CODE, null);
    				if(0 == code)
    				{
    					ioctl(MB, FCANMB_SET_ID, 0x4e2123 | FCAN_ID_EXT);	
    					ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
    					
    					// Unlock mailboxes
    					ioctl(FCAN, FCAN_UNLOCK_ALL_MB, null);	
    				}
    			}
    			i8CANTest = 0;
    			break;
    		}
    		case 4:
    		{
    			//CAN_TXVoltage();
    			CAN_TXStatus();
    			i8CANTest = 0;
    			break;
    		}
    		default:
    		{
    		    i8CANTest = 0;
    		    break;
    		}
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
    	
    	// Reinitialize driver?
    	if(REINIT_DRV8301)
    	{
    		REINIT_DRV8301 = 0;
    		/*
			// Initialise MOSFET driver
			if(0 != InitDRV8301(0,31,1))
			{
				// Driver did not initialise properly
				DRV8301_CONFIGURED = 0;
				SYSTEM.systemState = SYSTEM_FAULT;
			}
			else
			{
				// After everything is initialised, DRV8301_CONFIGURED = 1 moves system state from boot to init.
				DRV8301_CONFIGURED = 1;
			}
			*/
    	}

         /* feed the watchdog periodically */
         ioctl(COP, COP_CLEAR_COUNTER, NULL);

    }
}
