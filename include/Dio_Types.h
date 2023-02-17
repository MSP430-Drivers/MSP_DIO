/**********************************************************************************************************************
 * \file Dio_Types.h
 *
 * \author  Marco Aguilar
 *
 * \date Jul-16-2022
 *
 * \version 1.0 \n \n
 *
 * This file contains:
 *  - Definitions
 *  - Types
 *  - Interface Prototypes
 *  which are relevant for the MSP's project
 *********************************************************************************************************************/

#ifndef DIO_TYPES_H
#define DIO_TYPES_H

#include <Std_Types.h>

#define OK  (uint8)0x00;
#define NOK (uint8)0x01;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  bit0 = 1u,
  bit1 = 2u,
  bit2 = 4u,
  bit3 = 8u,
  bit4 = 16u,
  bit5 = 32u,
  bit6 = 64u,
  bit7 = 128u,
  bND  = 0xFF // No defined
} t_PortPin;

typedef enum
{
  port1 = 0,
  port2 = 1,
  pND   = 0xFF // No defined
} t_Port;

typedef enum
{
  input  = 0,
  output = 1,
  dND    = 0xFF // No defined
} t_PinDir;

typedef enum
{
  gpio      = 0,
  primary   = 1,
  secondary = 2,
  fND       = 0xFF // No defined
} t_FunctionSelect;

typedef enum
{
  low  = 0,
  high = 1
} t_PinState;

typedef enum
{
  disabled = 0,
  pullUp   = 1,
  pullDown = 2

} t_Ren;

#ifdef __cplusplus
}
#endif

#endif
