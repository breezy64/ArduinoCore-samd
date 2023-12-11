/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <sam.h>
#include <stdbool.h>
#include "board_definitions.h"

#define SYSCTRL_FUSES_OSC32K_CAL_ADDR   (NVMCTRL_OTP4 + 4)
#define SYSCTRL_FUSES_OSC32K_CAL_Pos   6
#define   SYSCTRL_FUSES_OSC32K_ADDR   SYSCTRL_FUSES_OSC32K_CAL_ADDR
#define   SYSCTRL_FUSES_OSC32K_Pos   SYSCTRL_FUSES_OSC32K_CAL_Pos
#define   SYSCTRL_FUSES_OSC32K_Msk   (0x7Fu << SYSCTRL_FUSES_OSC32K_Pos)

volatile bool g_interrupt_enabled = true;

static void gclk_sync(void) {
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;
}

static void dfll_sync(void) {
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0)
        ;
}


void board_init(void) {
  NVMCTRL->CTRLB.bit.RWS = 1;
  
  /* This works around a quirk in the hardware (errata 1.2.1) -
    the DFLLCTRL register must be manually reset to this value before
    configuration. */
  while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

    /* Write the coarse and fine calibration from NVM. */
  uint32_t coarse =
      ((*(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  uint32_t fine =
      ((*(uint32_t*)FUSES_DFLL48M_FINE_CAL_ADDR) & FUSES_DFLL48M_FINE_CAL_Msk) >> FUSES_DFLL48M_FINE_CAL_Pos;

  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  /* Wait for the write to finish. */
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {};

  SYSCTRL->DFLLCTRL.reg |=
    /* Enable USB clock recovery mode */
    SYSCTRL_DFLLCTRL_USBCRM |
    /* Disable chill cycle as per datasheet to speed up locking.
       This is specified in section 17.6.7.2.2, and chill cycles
       are described in section 17.6.7.2.1. */
    SYSCTRL_DFLLCTRL_CCDIS;

  /* Configure the DFLL to multiply the 1 kHz clock to 48 MHz */
  SYSCTRL->DFLLMUL.reg =
      /* This value is output frequency / reference clock frequency,
        so 48 MHz / 1 kHz */
      SYSCTRL_DFLLMUL_MUL(48000) |
      /* The coarse and fine values can be set to their minimum
        since coarse is fixed in USB clock recovery mode and
        fine should lock on quickly. */
      SYSCTRL_DFLLMUL_FSTEP(1) |
      SYSCTRL_DFLLMUL_CSTEP(1);

  SYSCTRL->DFLLCTRL.bit.MODE = 1;

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.bit.ENABLE = 1;

  /* Wait for the write to complete */
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY) {};

    /* Setup GCLK0 using the DFLL @ 48 MHz */
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_ID(0) |
      GCLK_GENCTRL_SRC_DFLL48M |
      /* Improve the duty cycle. */
      GCLK_GENCTRL_IDC |
      GCLK_GENCTRL_GENEN;

  /* Wait for the write to complete */
  while(GCLK->STATUS.bit.SYNCBUSY) {};

  SysTick_Config(1000);
    // Uncomment these two lines to output GCLK0 on the SWCLK pin.
    // PORT->Group[0].PINCFG[30].bit.PMUXEN = 1;
    // Set the port mux mask for odd processor pin numbers, PA30 = 30 is even number, PMUXE = PMUX Even
    // PORT->Group[0].PMUX[30 / 2].reg |= PORT_PMUX_PMUXE_H;
}
