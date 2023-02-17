/**********************************************************************************************************************
 * \file Dio_Driver.c
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
 *  which are relevant for the DIO Driver
 *********************************************************************************************************************/

#include <Dio_Driver.h>
#include <MSP430_Registers.h>

/// DIO driver has not been initialized
#define NOT_INITIALIZED (uint8)0u
/// DIO driver has been initialized
#define INITIALIZED (uint8)1u

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * DIO_t_getPinDir
 * Get pin Direction
 *
 * \param [in] e_port: Port 1 or 2 of MSP430g2553 \n
 *                     data_type  = t_Port \n
 *                     resolution = port1 / port2
 * \param [in] e_pin:  Pin port \n
 *                     data_type  = t_PortPin \n
 *                     resolution = bit0 to bit7 \n
 *
 * \return  data_type  = t_PinDir \n
 *          resolution = input / output
 *
 */
static t_PinDir DIO_t_getPinDir(t_Port e_port, t_PortPin e_pin);

/**
 *
 * DIO_t_getPinFunction
 * Get pin state
 *
 * \param [in] e_port: Port 1 or 2 of MSP430g2553 \n
 *                     data_type  = t_Port \n
 *                     resolution = port1 / port2
 * \param [in] e_pin:  Pin port \n
 *                     data_type  = t_PortPin \n
 *                     resolution = bit0 to bit7 \n
 *
 * \return  data_type  = t_FunctionSelect \n
 *          resolution = gpio/ secondary / primary
 *
 */
static t_FunctionSelect DIO_t_getPinFunction(t_Port e_port, t_PortPin e_pin);

/**
 *
 * DIO_v_setChannelFunction
 * Set Pin's function
 *
 * \param [in] e_pin:       Pin port \n
 *                          data_type  = t_PortPin \n
 *                          resolution = bit0 to bit7 \n
 * \param [in] p_sel1:      Pin port Sel1 \n
 *                          data_type  = uint8* \n
 *                          resolution = uint16 \n
 * \param [in] p_sel2:      Pin port Sel1 \n
 *                          data_type  = uint8* \n
 *                          resolution = uint16 \n
 * \param [in] e_function:  Dio functionality \n
 *                          data_type  = t_FunctionSelect \n
 *                          resolution = gpio/ secondary / primary \n
 *
 *
 */
static void DIO_v_setChannelFunction(t_PortPin e_pin, uint8* p_sel1, uint8* p_sel2, t_FunctionSelect e_function);

/**
 *
 * DIO_v_setChannelDir
 * Set Pin's direction
 *
 * \param [in] e_pin:       Pin port \n
 *                          data_type  = t_PortPin \n
 *                          resolution = bit0 to bit7 \n
 * \param [in] p_dirAddr:   Pin port dir ADDR \n
 *                          data_type  = uint8* \n
 *                          resolution = uint16 \n
 * \param [in] e_dir:       Pin direction \n
 *                          data_type  = t_PinDir \n
 *                          resolution = input / output \n
 *
 *
 */
static void DIO_v_setChannelDir(t_PortPin e_pin, uint8* p_dirAddr, t_PinDir e_dir);

/**
 *
 * DIO_v_setPinRes
 * Enable or disable internal resistor
 *
 * \param [in] e_pin:  Pin port \n
 *                     data_type  = t_PortPin \n
 *                     resolution = bit0 to bit7 \n
 * \param [in] e_port: Port 1 or 2 of MSP430g2553 \n
 *                     data_type  = t_Port \n
 *                     resolution = port1 / port2
 * \param [in] e_dir:  Pin direction \n
 *                     data_type  = t_PinDir \n
 *                     resolution = input / output \n
 * \param [in] e_res:  Set resistor internal \n
 *                     data_type  = t_Ren \n
 *                     resolution = disabled / pullUp / pullDown
 *
 */
static void DIO_v_setPinRes(t_PortPin e_pin, t_Port e_port, t_PinDir e_dir, t_Ren e_res);

/// Flag to indicate a DIO initialization
static uint8 DioInitialized = NOT_INITIALIZED;
/// Port Pin that interacts with other modules
t_PortPinStatus portPinStatus[PORTPIN_SIZE];

void DIO_v_init(void)
{
  const uint8 allPinsPort = 0xFF;
  uint8       index;

  // t_PortPinStatus* portPinStatusTmp;

  if(DioInitialized == INITIALIZED)
  {
    // TODO send a SW error (deinitilized)
  }
  else
  {
    // Required to function DIO_u_configPin can be used
    DioInitialized = INITIALIZED;

    // Initialize struct with default value 0xFF
    for(index = 0; index < PORTPIN_SIZE; index++)
    {
      portPinStatus[index].Port     = pND;
      portPinStatus[index].Pin      = bND;
      portPinStatus[index].Dir      = dND;
      portPinStatus[index].Function = fND;
    }
  }

  // Low consumption for unused pins
  P1DIR = input;
  P2DIR = input;
  P1OUT = input;
  P2OUT = input;

  // Datashet recomend enable Internal resistor
  P1REN = allPinsPort;
  P2REN = allPinsPort;
}

/// \brief
/// PxSEL = 0 and PxSEL2 = 0 -> I/O function is selected. \n
/// PxSEL = 1 and PxSEL2 = 0 -> Primary peripheral module function is selected. \n
/// PxSEL = 1 and PxSEL2 = 1 -> Secondary peripheral module function is selected. \n
static void DIO_v_setChannelFunction(t_PortPin e_pin, uint8* p_sel1, uint8* p_sel2, t_FunctionSelect e_function)
{
  switch(e_function)
  {
    case gpio:
      *p_sel1 &= ~e_pin;
      *p_sel2 &= ~e_pin;
      break;
    case primary:
      *p_sel1 |= e_pin;
      *p_sel2 &= ~e_pin;
      break;
    case secondary:
      *p_sel1 |= e_pin;
      *p_sel2 |= e_pin;
      break;
    default:
      break;
  }
}

// input -> 0
// output -> 1
static void DIO_v_setChannelDir(t_PortPin e_pin, uint8* p_dirAddr, t_PinDir e_dir)
{
  if(e_dir == input)
  {
    *p_dirAddr &= ~e_pin;
  }
  else
  {
    *p_dirAddr |= e_pin;
  }
}

static t_FunctionSelect DIO_t_getPinFunction(t_Port e_port, t_PortPin e_pin)
{
  uint8            pxsel;
  uint8            pxsel2;
  t_FunctionSelect statePin;

  switch(e_port)
  {
    case port1:
      pxsel  = *(uint8*)P1SEL_ADDR & e_pin;
      pxsel2 = *(uint8*)P1SEL2_ADDR & e_pin;

      // PxSEL = 0 and PxSEL2 = 0 -> I/O function
      if((pxsel == 0u) && (pxsel2 == 0u))
      {
        statePin = gpio;
      }
      // PxSEL = 1 and PxSEL2 = 0 -> Primary peripheral module
      else if((pxsel == e_pin) && (pxsel2 == 0u))
      {
        statePin = primary;
      }
      // PxSEL = 1 and PxSEL2 = 1 -> Secondary peripheral module
      else
      {
        statePin = secondary;
      }
      break;

    case port2:
      pxsel  = *(uint8*)P2SEL_ADDR & e_pin;
      pxsel2 = *(uint8*)P2SEL2_ADDR & e_pin;

      // PxSEL = 0 and PxSEL2 = 0 -> I/O function
      if((pxsel == 0u) && (pxsel2 == 0u))
      {
        statePin = gpio;
      }
      // PxSEL = 1 and PxSEL2 = 0 -> Primary peripheral module
      else if((pxsel == e_pin) && (pxsel2 == 0u))
      {
        statePin = primary;
      }
      // PxSEL = 1 and PxSEL2 = 1 -> Secondary peripheral module
      else
      {
        statePin = secondary;
      }
      break;

    default:
      break;
  }

  return statePin;
}

static t_PinDir DIO_t_getPinDir(t_Port e_port, t_PortPin e_pin)
{
  t_PinDir retVal;
  switch(e_port)
  {
    case port1:
      // output -> 1
      if((*(uint8*)P1DIR_ADDR & e_pin) == e_pin)
      {
        retVal = output;
      }
      // input -> 0
      else
      {
        retVal = input;
      }
      break;

    case port2:
      // output -> 1
      if((*(uint8*)P2DIR_ADDR & e_pin) == e_pin)
      {
        retVal = output;
      }
      // input -> 0
      else
      {
        retVal = input;
      }
      break;

    default:
      break;
  }

  return retVal;
}

static void DIO_v_setPinRes(t_PortPin e_pin, t_Port e_port, t_PinDir e_dir, t_Ren e_res)
{
  switch(e_port)
  {
    case port1:
      // valid only for input pins
      if((e_dir == input) || (e_res != disabled))
      {
        P1REN &= ~e_pin;
      }
      else
      {
        // If the pin's pullup/pulldown resistor is enabled, the corresponding bit in the PxOUT register selects pullup or pulldown.
        // PXOUT 1 = pullUp
        // PXOUT 0 = pulldown
        P1REN |= e_pin;
        if(e_res == pullUp)
        {
          P1OUT |= e_pin;
        }
        else
        {
          P1OUT &= ~e_pin;
        }
      }
      break;

    case port2:
      // valid only for input pins
      if((e_dir == input) || (e_res != disabled))
      {
        P2REN &= ~e_pin;
      }
      else
      {
        // If the pin's pullup/pulldown resistor is enabled, the corresponding bit in the PxOUT register selects pullup or pulldown.
        // PXOUT 1 = pullUp
        // PXOUT 0 = pulldown
        P2REN |= e_pin;
        if(e_res == pullUp)
        {
          P2OUT |= e_pin;
        }
        else
        {
          P2OUT &= ~e_pin;
        }
      }
      break;

    default:
      break;
  }
}

uint8 DIO_u_configPin(t_PortPin e_pin, t_Port e_port, t_PinDir e_dir, t_FunctionSelect e_function, t_Ren e_res)
{
  uint8 retVal = OK;

  if(DioInitialized == INITIALIZED)
  {
    switch(e_port)
    {
      case port1:
        DIO_v_setChannelDir(e_pin, (uint8*)P1DIR_ADDR, e_dir);
        DIO_v_setChannelFunction(e_pin, (uint8*)P1SEL_ADDR, (uint8*)P1SEL2_ADDR, e_function);
        break;

      case port2:
        DIO_v_setChannelDir(e_pin, (uint8*)P2DIR_ADDR, e_dir);
        DIO_v_setChannelFunction(e_pin, (uint8*)P2SEL_ADDR, (uint8*)P2SEL2_ADDR, e_function);
        break;

      default:
        break;
    }
    DIO_v_setPinRes(e_pin, e_port, e_dir, e_res);
    portPinStatus[DIO_u_getPinIndex(e_pin, e_port)].Dir      = e_dir;
    portPinStatus[DIO_u_getPinIndex(e_pin, e_port)].Port     = e_port;
    portPinStatus[DIO_u_getPinIndex(e_pin, e_port)].Pin      = e_pin;
    portPinStatus[DIO_u_getPinIndex(e_pin, e_port)].Function = e_function;
  }
  else
  {
    retVal = NOK;
  }

  return retVal;
}

uint8 DIO_u_setPinState(t_PortPin e_pin, t_Port e_port, t_PinState u_state)
{
  t_FunctionSelect statePin;
  t_PinDir         dirPin;
  uint8            retVal = OK;

  statePin = DIO_t_getPinFunction(e_port, e_pin);
  dirPin   = DIO_t_getPinDir(e_port, e_pin);

  // Valid only for pins that are configured as GPIO and output
  if((statePin == gpio) && (dirPin == output))
  {
    switch(e_port)
    {
      case port1:
        if(u_state == high)
        {
          P1OUT |= e_pin;
        }
        else
        {
          P1OUT &= ~e_pin;
        }
        break;

      case port2:
        if(u_state == high)
        {
          P2OUT |= e_pin;
        }
        else
        {
          P2OUT &= ~e_pin;
        }
        break;

      default:
        break;
    }
  }
  else
  {
    retVal = NOK;
  }

  return retVal;
}

uint8 DIO_u_getPinState(t_PortPin e_pin, t_Port e_port, t_PinState* p_pinState)
{
  uint8            retVal = OK;
  t_PinDir         dirPin;
  t_FunctionSelect statePin;

  statePin = DIO_t_getPinFunction(e_port, e_pin);
  dirPin   = DIO_t_getPinDir(e_port, e_pin);

  // Valid only for pins that are configured as GPIO and input
  if((statePin == gpio) && (dirPin == input))
  {
    switch(e_port)
    {
      case port1:
        if((P1IN & e_pin) == e_pin)
        {
          *p_pinState = high;
        }
        else
        {
          *p_pinState = low;
        }
        break;

      case port2:
        if((P2IN & e_pin) == e_pin)
        {
          *p_pinState = high;
        }
        else
        {
          *p_pinState = low;
        }
        break;

      default:
        break;
    }
  }
  else
  {
    retVal = NOK;
  }

  return retVal;
}

uint8 DIO_u_getPinIndex(t_PortPin e_pin, t_Port e_port)
{
  uint8       bit        = 1u;
  uint8       done       = 0u;
  uint8       index      = 0u;
  const uint8 bytelength = 8u;

  while((index < bytelength) && (done == 0u))
  {
    if(bit == e_pin) // obtain bit position
    {
      done = 1u;
    }
    else
    {
      // move to next bit from LSB to MSB
      bit = bit << 1u;
      index++;
    }
  }

  if(e_port)
  {
    // Add port2 offset
    index += bytelength;
  }

  return index;
}

#ifdef __cplusplus
}
#endif
