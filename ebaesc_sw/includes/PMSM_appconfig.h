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
#define I_MAX (330.0)  
#define U_DCB_MAX (42.6191489)  
#define U_MAX (42.6191489)  
#define N_MAX (20000.0)  
#define E_MAX (42.6191489) 
/*
#define I_MAX                           (164.8)
#define U_DCB_MAX                       (36.3)
#define U_MAX                           (21.0)
#define N_MAX                           (60000.0)
#define FREQ_MAX                        (1000.0)
#define E_MAX                           (36.3)
*/
#define U_DCB_TRIP                      FRAC16(0.798898071625)
#define U_DCB_UNDERVOLTAGE              FRAC16(0.247933884298)
#define U_DCB_OVERVOLTAGE               FRAC16(0.798898071625)
#define N_OVERSPEED                     FRAC16(0.954545454545)
#define N_MIN                           FRAC16(0.013636363636)
#define N_NOM                           FRAC16(0.909090909091)
#define I_PH_NOM                        FRAC16(0.121359223301)

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
#define D_KP_GAIN                       FRAC16(0.625)
#define D_KP_SHIFT                      (1)
#define D_KI_GAIN                       FRAC16(0.999969482421875)
#define D_KI_SHIFT                      (-2)
//Q-axis Controller - Parallel type     
#define Q_KP_GAIN                       FRAC16(0.625)
#define Q_KP_SHIFT                      (1)
#define Q_KI_GAIN                       FRAC16(0.999969482421875)
#define Q_KI_SHIFT                      (-2)

//Speed Loop Control                    
//----------------------------------------------------------------------
//Loop Bandwidth                        = 12 [Hz]
//Loop Attenuation                      = 0.707 [-]
//Loop sample time                      = 0.000125 [sec]
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#define SPEED_PI_PROP_GAIN              FRAC16(0.625)
#define SPEED_PI_PROP_SHIFT             (3)
#define SPEED_PI_INTEG_GAIN             FRAC16(0.639984)
#define SPEED_PI_INTEG_SHIFT            (-6)
#define SPEED_LOOP_HIGH_LIMIT           FRAC16(0.09)
#define SPEED_LOOP_LOW_LIMIT            FRAC16(-0.09)


#define SPEED_RAMP_UP                   FRAC16(0.001)//FRAC16(0.000056818182)
#define SPEED_RAMP_DOWN                 FRAC16(0.001)//FRAC16(0.000056818182)

#define SPEED_LOOP_CNTR                 (16)	// 62.5 usec interval
#define SPEED_LOOP_FREQ                 (1000)



//Sensorless BEMF DQ nad Tracking Observer
//----------------------------------------------------------------------
//Loop Bandwidth                        = 280 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.000125 [sec]
//----------------------------------------------------------------------

//Bemf DQ Observer    
/*
 // Aerodrive 500
#define I_SCALE 						FRAC16(0.785829307569)  
#define U_SCALE 						FRAC16(0.512991885935)  
#define E_SCALE 						FRAC16(0.886743117115)  
#define WI_SCALE 						FRAC16(0.617188894909)  
#define I_SCALE_SHIFT 					(1)  
#define BEMF_DQ_KP_GAIN 				FRAC16(0.592979347366)  
#define BEMF_DQ_KP_SHIFT 				(-2)  
#define BEMF_DQ_KI_GAIN 				FRAC16(0.68571903253)  
#define BEMF_DQ_KI_SHIFT 				(-4)  
*/


// TMC

/*
#define I_SCALE FRAC16(0.879120879121)  
#define U_SCALE FRAC16(0.101471647285)  
#define E_SCALE FRAC16(0.101471647285)  
#define WI_SCALE FRAC16(0.443045117814)  
*/
/*
#define BEMF_DQ_KP_GAIN FRAC16(0.713989175715)  
#define BEMF_DQ_KP_SHIFT (0)  
#define BEMF_DQ_KI_GAIN FRAC16(0.837972381373)  
#define BEMF_DQ_KI_SHIFT (-3) 
*/

// EMAX

#define I_SCALE FRAC16(0.554913294798)  
#define U_SCALE FRAC16(0.735680504467)  
#define E_SCALE FRAC16(0.735680504467)  
#define WI_SCALE FRAC16(0.07263798043) 

#define BEMF_DQ_KP_GAIN FRAC16(0.516137807798)  
#define BEMF_DQ_KP_SHIFT (0)  
#define BEMF_DQ_KI_GAIN FRAC16(0.670005144485)  
#define BEMF_DQ_KI_SHIFT (-8) 



//Bemf DQ Observer   
/*
#define TO_KP_GAIN 						FRAC16(0.767975)  
#define TO_KP_SHIFT 					(-2)  
#define TO_KI_GAIN 						FRAC16(0.299988)  
#define TO_KI_SHIFT 					(-10)  
#define TO_THETA_GAIN 					FRAC16(0.5)  
#define TO_THETA_SHIFT 					(-1)  
*/

//EMAX
#define TO_KP_GAIN 						FRAC16(0.767975)  
#define TO_KP_SHIFT 					(-2)  
#define TO_KI_GAIN 						FRAC16(0.299988)  
#define TO_KI_SHIFT 					(-10)  
#define TO_THETA_GAIN 					FRAC16(0.4)  
#define TO_THETA_SHIFT 					(1)  



#endif

//End of generated file                 
/**********************************************************************/
