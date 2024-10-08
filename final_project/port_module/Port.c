 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: sherif mokhtar 
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"



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
#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif
#endif
static const *Port_pin_config=NULL_PTR;
static uint8 Port_status = PORT_NOT_INITIALIZED ;

void Port_Init(const Port_ConfigType *ConfigPtr )
{
   #if (PORT_DEV_ERROR_DETECT == STD_ON)
   if ( ConfigPtr == NULL_PTR )
   {
Det_ReportError( PORT_MODULE_ID ,
                                PORT_INSTANCE_ID ,
                               PORT_INIT ,
		                PORT_E_PARAM_CONFIG );

   }
   else
   #endif 
   {
    Port_status=PORT_INITIALIZED;
    Port_pin_config=ConfigPtr->Pins_Config;  

   }
 volatile uint32 * PortGpio_Ptr = NULL_PTR;
   volatile unsigned long delay=0;
  SYSCTL_REGCGC2_REG|=PORT_ENABLE_CLK; 
  delay=SYSCTL_REGCGC2_REG;
for(uint8 Counter=0;Counter<PIN_MAXIMUN_CHANNEL_NUM;Counter++)
{
   
   switch( Port_pin_config[Counter].Port_num ) 
{
        case  PORTA_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PORTB_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PORTC_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PORTD_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PORTE_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PORTF_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
}

    if( (( Port_pin_config[Counter].Port_num == 3) && ( Port_pin_config[Counter].Pin_num == 7)) || (( Port_pin_config[Counter].Port_num == 5) && ( Port_pin_config[Counter].Pin_num == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) ,  Port_pin_config[Counter].Pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( ( Port_pin_config[Counter].Port_num == 2) && ( Port_pin_config[Counter].Pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }


if( Port_pin_config[Counter].direction==PORT_PIN_IN)
{ //Configure Pin as input pin
       CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) ,Port_pin_config[Counter].Pin_num );
       //if input pin configure resistor according to sent configuration
       if(Port_pin_config[Counter].resistor==PULL_UP)
       { //enable pull up resistance for input pin 
        SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_pin_config[Counter].Pin_num); 
       }
       else if(Port_pin_config[Counter].resistor==PULL_DOWN)
       {//enable pull down resistance for input pin
         SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_pin_config[Counter].Pin_num); 
       }
       else
       {
          // do nothing
       }
       

}
 else if(Port_pin_config[Counter].direction==PORT_PIN_OUT)
  {//Configure Pin as output pin
     SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_pin_config[Counter].Pin_num ); 
     if(Port_pin_config[Counter].Init_Value==STD_LOW)
     {
       CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_pin_config[Counter].Pin_num );
     }
     else if(Port_pin_config[Counter].Init_Value==STD_HIGH) 
     {
       SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_pin_config[Counter].Pin_num );
     }
     else{
       // do nothing
     }

  }
  else
  {
    // do nothing
  }

  if(Port_pin_config[Counter].Pin_mode!=DIO_Mode)//if Pin is going to work as an alternative function not Gpio
    { 
       SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_pin_config[Counter].Pin_num);   
    }
    else if(Port_pin_config[Counter].Pin_mode==DIO_Mode)//if Pin is going to work as  Gpio
    {
      CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_pin_config[Counter].Pin_num);
    }

     else if((Port_pin_config[Counter].Pin_mode)==ADC_Mode)
    { // disable digital enable for this Pin
      CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Port_pin_config[Counter].Pin_num));
      //enable analog mode
      SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Port_pin_config[Counter].Pin_num));
      
    }
    else if((Port_pin_config[Counter].Pin_mode)!=ADC_Mode)
    {   // enable digital register for all GPIO or Alternative pins
       SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Port_pin_config[Counter].Pin_num));
       //disable analog mode
       CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Port_pin_config[Counter].Pin_num));
    }
    else
    {
     // do nothing 
    }
    /*writing pin mode inside Control register*/
 *(volatile uint32 *)(( volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) =  (*(volatile uint32 *)(( volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET)&~(0x0000000F<<((Port_pin_config[Counter].Pin_num)*4))) |((Port_pin_config[Counter].Pin_mode)<<((Port_pin_config[Counter].Pin_num)*4));
    
   
}
}

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
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
#endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
        if(Port_status==PORT_NOT_INITIALIZED){
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT); 
        }
        else
        {
          //do nothing
        }
       /*check for maxium pins number*/
	if (Pin >= PIN_MAXIMUN_CHANNEL_NUM)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
	}
	else
	{
	 // do nothing
        }
        /*check if direction is changeable*/
         if(Port_pin_config[Pin].Dir_Change==NO_CHANGEABLE)
         {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE); 
        }
        else
        {
          //do nothing
        }
       
  #endif
     
    volatile uint32 * PortGpio_Ptr = NULL_PTR;  /* point to the required Port Registers base address */
 
  
        switch(Port_pin_config[Pin].Port_num)
{
        case  PORTA_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PORTB_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PORTC_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PORTD_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PORTE_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PORTF_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
}
  
if( ((Port_pin_config[Pin].Port_num == PORTD_ID) && (Port_pin_config[Pin].Pin_num == PIN7_ID)) || ((Port_pin_config[Pin].Port_num == PORTF_ID) && (Port_pin_config[Pin].Pin_num == PIN0_ID)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = UNLOCK_MAGICAL_NUMBER;                     /* Unlock the GPIOCR register */   
        SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_pin_config[Pin].Pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (Port_pin_config[Pin].Port_num == PORTC_ID) && (Port_pin_config[Pin].Pin_num <= PIN3_ID) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
    if(Direction==PORT_PIN_IN)
      { //Configure Pin as input pin
       CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_pin_config[Pin].Pin_num);
       
    }
    else if(Direction==PORT_PIN_OUT)
    {//Configure Pin as output pin
     SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_pin_config[Pin].Pin_num); 
    }
     else{
       // do nothing
     }
    }
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
void Port_RefreshPortDirection(void)
{
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
        if(Port_status==PORT_NOT_INITIALIZED){
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
		PORT_SET_PIN_MODE_SID, PORT_E_UNINIT); 
        }
        else
        {
          //do nothing
        }
#endif
        /* function variables used inside function to store data from pointer to strucute*/
  
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
        for(uint8 Counter=0;Counter<PIN_MAXIMUN_CHANNEL_NUM;Counter++)
        {
          
  
           switch( Port_pin_config[Counter].Port_num)
{
        case  PORTA_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PORTB_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PORTC_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PORTD_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PORTE_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PORTF_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
}
  
if( (( Port_pin_config[Counter].Port_num == PORTD_ID) && ( Port_pin_config[Counter].Pin_num == PIN7_ID)) || (( Port_pin_config[Counter].Port_num == PORTF_ID) && ( Port_pin_config[Counter].Pin_num == PIN0_ID)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = UNLOCK_MAGICAL_NUMBER;                     /* Unlock the GPIOCR register */   
        SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) ,  Port_pin_config[Counter].Pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( ( Port_pin_config[Counter].Port_num == PORTC_ID) && ( Port_pin_config[Counter].Pin_num <= PIN3_ID) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
     if( Port_pin_config[Counter].direction==PORT_PIN_IN)
      { //Configure Pin as input pin
       CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) ,  Port_pin_config[Counter].Pin_num);
    }
    else if( Port_pin_config[Counter].direction==PORT_PIN_OUT)
    {//Configure Pin as output pin
     SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) ,  Port_pin_config[Counter].Pin_num); 
    }
     else{
       // do nothing
     }
    }   
        }
        

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
#if (PORT_SET_PIN_MODE_API == STD_ON)

void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
       #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
        if(Port_status==PORT_NOT_INITIALIZED){
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_UNINIT); 
        }
        else
        {
          //do nothing
        }
       /*check for maxium pins number*/
	if (Pin >= PIN_MAXIMUN_CHANNEL_NUM)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
	}
	else
	{
	 // do nothing
        }
        /*check if mode is changeable*/
        if(Port_pin_config[Pin].Mode_Change==NO_CHANGEABLE)
        {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE); 
        }
        else
        {
          //do nothing
        }
        if(Mode>CAN_Mode)
        {
          Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        }
        else
        {
           //do nothing 
        }
       
  #endif
        
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
 
    
     switch(Port_pin_config[Pin].Port_num)
{
        case  PORTA_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  PORTB_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  PORTC_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  PORTD_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  PORTE_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  PORTF_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
}
  
if( ((Port_pin_config[Pin].Port_num == PORTD_ID) && (Port_pin_config[Pin].Pin_num == PIN7_ID)) || ((Port_pin_config[Pin].Port_num == PORTF_ID) && (Port_pin_config[Pin].Pin_num == PIN0_ID)) ) /* PD7 or PF0 */
    {
        *( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = UNLOCK_MAGICAL_NUMBER;                     /* Unlock the GPIOCR register */   
        SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_pin_config[Pin].Pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
   else if( (Port_pin_config[Pin].Port_num == PORTC_ID) && (Port_pin_config[Pin].Pin_num <= PIN3_ID) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
    if(Mode!=DIO_Mode)//if Pin is going to work as an alternative function not Gpio
    { 
       SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_pin_config[Pin].Pin_num);   
    }
    else if(Mode==DIO_Mode)//if Pin is going to work as  Gpio
    {
      CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_pin_config[Pin].Pin_num);
    }

     else if(Mode==ADC_Mode)
    { // disable digital enable for this Pin
      CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Port_pin_config[Pin].Pin_num));
      //enable analog mode
      SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Port_pin_config[Pin].Pin_num));
      
    }
    else if(Mode!=ADC_Mode)
    {   // enable digital register for all GPIO or Alternative pins
       SET_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Port_pin_config[Pin].Pin_num));
       //disable analog mode
       CLEAR_BIT(*( uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Port_pin_config[Pin].Pin_num));
    }
    else
    {
     // do nothing 
    }
    *(volatile uint32 *)(( volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) = (*(volatile uint32 *)(( volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &~(0x0000000F<<(Port_pin_config[Pin].Pin_num*4))) |(Mode<<(Port_pin_config[Pin].Pin_num*4));
   
        }
    #endif        