/**********************************************************************/
// File Name: {FM_project_loc}/../PMSM_appconfig.h 
//
// Date:  12. October, 2014
//
// Automatically generated file for static configuration of the PMSM FOC application
/**********************************************************************/

#ifndef __M1_PMSMFOC_CONFIG_SETUP_H
#define __M1_PMSMFOC_CONFIG_SETUP_H


//Motor Parameters                      
//----------------------------------------------------------------------
//Pole-pair number                      = 6 [-]
//Stator resistance                     = 0.0266 [Ohms]
//Direct axis inductance                = 0.0000122 [H]
//Quadrature axis inductance            = 0.0000122 [H]
//Back-EMF constant                     = 0.019 [V.sec/rad]
//Drive inertia                         = 0.000001 [kg.m2]
//Nominal current                       = 20 [A]

#define MOTOR_PP                        (6)
//----------------------------------------------------------------------

//Application scales                    
//----------------------------------------------------------------------
#define I_MAX                           (164.8)
#define U_DCB_MAX                       (36.3)
#define U_MAX                           (21.0)
#define N_MAX                           (60000.0)
#define FREQ_MAX                        (1000.0)
#define E_MAX                           (36.3)
#define U_DCB_TRIP                      FRAC16(0.798898071625)
#define U_DCB_UNDERVOLTAGE              FRAC16(0.247933884298)
#define U_DCB_OVERVOLTAGE               FRAC16(0.798898071625)
#define N_OVERSPEED                     FRAC16(0.954545454545)
#define N_MIN                           FRAC16(0.013636363636)
#define N_NOM                           FRAC16(0.909090909091)
#define I_PH_NOM                        FRAC16(0.121359223301)
//DCB Voltage Filter                    
#define UDCB_IIR_B0                     FRAC16(0.004723256666)
#define UDCB_IIR_B1                     FRAC16(0.004723256666)
#define UDCB_IIR_A1                     FRAC16(-0.115553486669)
//Mechanical Alignment                  
#define ALIGN_VOLTAGE                   FRAC16(0.028571428571)
#define ALIGN_DURATION                  (8000)

//Current Loop Control                  
//----------------------------------------------------------------------
//Loop Bandwidth                        = 450 [Hz]
//Loop Attenuation                      = 0.8 [-]
//Loop sample time                      = 0.000125 [sec]
//----------------------------------------------------------------------
//Current Controller Output Limit       
#define CLOOP_LIMIT                     FRAC16(0.9)
//D-axis Controller - Parallel type     
#define D_KP_GAIN                       FRAC16(0.897500791784)
#define D_KP_SHIFT                      (-2)//-2
#define D_KI_GAIN                       FRAC16(0.765389513237)
#define D_KI_SHIFT                      (-5)//-5
//Q-axis Controller - Parallel type     
#define Q_KP_GAIN                       FRAC16(0.897500791784)
#define Q_KP_SHIFT                      (-2)//-2
#define Q_KI_GAIN                       FRAC16(0.765389513237)
#define Q_KI_SHIFT                      (-5)//-5

//Speed Loop Control                    
//----------------------------------------------------------------------
//Loop Bandwidth                        = 12 [Hz]
//Loop Attenuation                      = 0.707 [-]
//Loop sample time                      = 0.000125 [sec]
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#define SPEED_PI_PROP_GAIN              FRAC16(0.3285)
#define SPEED_PI_PROP_SHIFT             (0)
#define SPEED_PI_INTEG_GAIN             FRAC16(0.762935023406)
#define SPEED_PI_INTEG_SHIFT            (-7)
#define SPEED_LOOP_HIGH_LIMIT           FRAC16(0.121359223301)
#define SPEED_LOOP_LOW_LIMIT            FRAC16(-0.121359223301)


#define SPEED_RAMP_UP                   FRAC16(0.0002)//FRAC16(0.000056818182)
#define SPEED_RAMP_DOWN                 FRAC16(0.0002)//FRAC16(0.000056818182)

#define SPEED_LOOP_CNTR                 (16)	// 62.5 usec interval
#define SPEED_LOOP_FREQ                 (1000)

#define SPEED_IIR_B0                    FRAC16(0.00698853)
#define SPEED_IIR_B1                    FRAC16(0.00698853)
#define SPEED_IIR_A1                    FRAC16(-0.486023)

//Sensorless BEMF DQ nad Tracking Observer
//----------------------------------------------------------------------
//Loop Bandwidth                        = 280 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.000125 [sec]
//----------------------------------------------------------------------
//Bemf DQ Observer                      
#define I_SCALE 						FRAC16(0.785829307569)  
#define U_SCALE 						FRAC16(0.512991885935)  
#define E_SCALE 						FRAC16(0.886743117115)  
#define WI_SCALE 						FRAC16(0.617188894909)  
#define I_SCALE_SHIFT 					(1)  
#define BEMF_DQ_KP_GAIN 				FRAC16(0.592979347366)  
#define BEMF_DQ_KP_SHIFT 				(-2)  
#define BEMF_DQ_KI_GAIN 				FRAC16(0.68571903253)  
#define BEMF_DQ_KI_SHIFT 				(-4)  

//Bemf DQ Observer                      
#define TO_KP_GAIN 						FRAC16(0.768000000000)  
#define TO_KP_SHIFT 					(-5)  
#define TO_KI_GAIN 						FRAC16(0.926493372655)  
#define TO_KI_SHIFT 					(-13)  
#define TO_THETA_GAIN 					FRAC16(0.5)  
#define TO_THETA_SHIFT 					(-1)  

//Observer speed output filter          
#define TO_SPEED_IIR_B0 				FRAC16(0.069607840518)  
#define TO_SPEED_IIR_B1 				FRAC16(0.069607840518)  
#define TO_SPEED_IIR_A1 				FRAC16(0.014215681036) 

//Open loop start-up                    
#define OL_START_RAMP_INC 				FRAC32(0.000045454545)  
#define OL_START_I 						FRAC16(0.015169902913)  
#define MERG_SPEED_TRH 					FRAC16(0.008333333333)  
#define MERG_COEFF 						FRAC16(0.007324218750)  


//Control Structure Module - Scalar Control
//----------------------------------------------------------------------
#define SCALAR_VHZ_FACTOR_GAIN          FRAC16(0.942857142857)
#define SCALAR_VHZ_FACTOR_SHIFT         (0)
#define SCALAR_INTEG_GAIN               FRAC16(0.1375)
#define SCALAR_INTEG_SHIFT              (0)
#define SCALAR_RAMP_UP                  FRAC32(0.000056818182)
#define SCALAR_RAMP_DOWN                FRAC32(0.000056818182)

#endif

//End of generated file                 
/**********************************************************************/
