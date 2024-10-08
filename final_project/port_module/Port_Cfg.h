 /******************************************************************************
 *
 * Module: Dio
 *
 * File Name: Dio_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - port Driver
 *
 * Author: sherif mokhtar
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for presence of PORT_Pin  Direction API */
#define PORT_SET_PIN_DIRECTION_API                (STD_ON)

/* Pre-compile option for enable or disable Port Pin mode  API */
#define PORT_SET_PIN_MODE_API              (STD_ON)

/* Number of the configured PORT Channels */
#define PIN_MAXIMUN_CHANNEL_NUM               (43U)

/*magic number to write in commit register*/
#define UNLOCK_MAGICAL_NUMBER 0x4C4F434BU

/*define  used to enable 6 ports clock */
#define PORT_ENABLE_CLK 0x003F

#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/*pins defination to be used in configuration*/
#define   PIN0_ID    (0U)
#define   PIN1_ID    (1U)
#define   PIN2_ID    (2U)
#define   PIN3_ID    (3U)
#define   PIN4_ID    (4U)
#define   PIN5_ID    (5U)
#define   PIN6_ID    (6U)
#define   PIN7_ID    (7U)

/*Portdefination to be used in configuration*/
#define   PORTA_ID    (0U)
#define   PORTB_ID    (1U)
#define   PORTC_ID    (2U)
#define   PORTD_ID    (3U)
#define   PORTE_ID    (4U)
#define   PORTF_ID    (5U)


#endif