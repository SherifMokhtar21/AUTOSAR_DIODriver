 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: sherif mokhtar
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PORT_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PORT_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port Init*/
#define  PORT_INIT                       (uint8)0x00

/* Service ID for Port_SetPinDirection  */
#define  PORT_SET_PIN_DIRECTION_SID      (uint8)0x01 

/* Service ID for Port_RefreshPortDirection */
#define PORT_REFRESH_PORT_DIRECTION_SID  (uint8)0x02  

/* Service ID for Port_GetVersionInfo*/
#define PORT_GET_VERSION_INFO_SID        (uint8)0x03  

/* Service ID for DIO read Channel Group */
#define PORT_SET_PIN_MODE_SID            (uint8)0x04 


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to Invalid Port Pin ID requested  */
#define PORT_E_PARAM_PIN                  (uint8)0x0A

/* DET code to report Port Pin not configured as changeable  */
#define PORT_E_DIRECTION_UNCHANGEABLE     (uint8)0x0B

/* API Port_Init service called with wrong parameter.  */
#define PORT_E_PARAM_CONFIG               (uint8)0x0C

/* DET code to report Invalid mode */
#define PORT_E_PARAM_INVALID_MODE         (uint8)0x0D

/*service called when  mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE           (uint8)0x0E 

/*API service called without module initialization  */
#define PORT_E_UNINIT                      (uint8)0x0F
/* APIs called with a Null  Pointer */
#define PORT_E_PARAM_POINTER               (uint8)0x10 


/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Description: Type definition for Port_PinType used by the PORT */

typedef volatile uint8 Port_PinType; 
/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to hold pin changeable */
typedef enum
{
    NO_CHANGEABLE,CHANGEABLE
}Pin_Changeable_Features;
/* Description: Enum to hold PIN initial value */
typedef enum
{
    LOW,HIGH
}Initial_Value;

/* Enum for Port_PinModeType used by the PORT APIs */
typedef enum
{
    DIO_Mode,UART_Mode,SSI_Mode,I2C_Mode,M0PWM_Mode,M1PWM_Mode,ADC_Mode,RESERVED,CAN_Mode
}Port_PinModeType;


/* Description: Structure to define port configuration structure:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 *      5. tinitial value that pin intializes with in case of output pin.
 *      6. what kind of mode you want pin to work at.
 *      7. the direction of pin is changeable or not.
 *      8. the pin mode is changeable or not.
 */
typedef struct  {
    uint8 Port_num; 
    uint8 Pin_num; 
    Port_PinDirectionType  direction;
    Port_InternalResistor resistor;
    Initial_Value Init_Value;
    Port_PinModeType Pin_mode  ;
    Pin_Changeable_Features Dir_Change;
    Pin_Changeable_Features Mode_Change;
}Pin_ConfigType;

/* Data Structure required for initializing the Pin Driver */
typedef struct Port_ConfigType
{
  Pin_ConfigType Pins_Config[PIN_MAXIMUN_CHANNEL_NUM];
}Port_ConfigType;



/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/


/*******************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: non-Reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
*******************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr );

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
*                : Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API  == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction ); 
#endif
/************************************************************************************
* Service Name: Port_SetPinMode 
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
*                : Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode. 
************************************************************************************/
#if (PORT_SET_PIN_MODE_API  == STD_ON)

void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );

#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void); 

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Returns the version information of this module.. 
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by PORT and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
