/*********************************************************************
*                SEGGER Microcontroller GmbH                         *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2021  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V6.26 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software  has been licensed to  Cypress Semiconductor Corporation,
whose registered  office is situated  at 198 Champion Ct. San Jose, CA 
95134 USA  solely for the  purposes of creating  libraries for Cypress
PSoC3 and  PSoC5 processor-based devices,  sublicensed and distributed
under  the  terms  and  conditions  of  the  Cypress  End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              Cypress Semiconductor Corp, 198 Champion Ct., San Jose, CA 95134, USA
Licensed SEGGER software: emWin
License number:           GUI-00319
License model:            Cypress Services and License Agreement, signed June 9th/10th, 2009
                          and Amendment Number One, signed June 28th, 2019 and July 2nd, 2019
                          and Amendment Number Two, signed September 13th, 2021 and September 18th, 2021
                          and Amendment Number Three, signed May 2nd, 2022 and May 5th, 2022
Licensed platform:        Any Cypress platform (Initial targets are: PSoC3, PSoC5)
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2009-06-12 - 2022-07-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include "GUI.h"
#include "LCDConf_CompactColor_16.h"

#include "GUIDRV_CompactColor_16.h"
#include "GUI_Type.h"

#include "cyhal.h"
#include "LCDConf.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/
//
// Physical display size
//   The display size should be adapted in order to match the size of
//   the target display.
//
#define XSIZE_PHYS 105
#define YSIZE_PHYS 160

//
// Color conversion
//   The color conversion functions should be selected according to
//   the color mode of the target display. Detaileds can be found in
//   the chapter "Colors" in the emWin user manual.
//
#define COLOR_CONVERSION GUICC_M565

//
// Display driver
//
#define DISPLAY_DRIVER GUIDRV_COMPACT_COLOR_16

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
  #error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
  #error No display driver defined!
#endif

/*********************************************************************
*
*       Display access functions
*
**********************************************************************
*/

cyhal_spi_t mSPI;

static const SPI_pins *pins;

void Display_WriteM8_A1(U8 *wrData, int numbytes)
{
    int i;
    cyhal_gpio_write(pins->dc, 1u);
    for (i = 0; i < numbytes; i++)
    {
        write_data(wrData[i]);
    }
}

void Display_WriteM8_A0(U8 *wrData, int numbytes)
{
    int i;
    cyhal_gpio_write(pins->dc, 0u);
    for (i = 0; i < numbytes; i++)
    {
        write_data(wrData[i]);
    }
}

void Display_ReadM8_A1(U8 *rdData, int numbytes)
{
  GUI_USE_PARA(rdData);
  GUI_USE_PARA(numbytes);
}

void Display_Write8_A0(U8 byte)
{
    cyhal_gpio_write(pins->dc, 0u);
    write_data(byte);
}

void Display_Write8_A1(U8 byte)
{
    cyhal_gpio_write(pins->dc, 1u);
    write_data(byte);
}

void write_data(uint8_t data)
{
    cyhal_spi_send(&mSPI,data);
}

uint8_t read_data(void)
{
    uint32_t receive_data = 0u;
    cyhal_spi_recv(&mSPI, &receive_data);
    return receive_data;
}

void SPI_st7735_write_reset_pin(bool value)
{
    cyhal_gpio_write(pins->rst, value);
}

cy_rslt_t SPI_init8(const SPI_pins *data)
{
    pins = data;
    cy_rslt_t rslt = cyhal_spi_init(&mSPI, pins->MOSI, pins->MISO, pins->SCK, pins->SS, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);
    if (CY_RSLT_SUCCESS == rslt)
    rslt = cyhal_spi_set_frequency(&mSPI, pins->frec);
    if (CY_RSLT_SUCCESS == rslt)
    rslt = cyhal_gpio_init(pins->dc, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0u);
    if (CY_RSLT_SUCCESS == rslt)
    rslt = cyhal_gpio_init(pins->rst, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1u);
    return rslt;
}
/*********************************************************************
*
*       ST7735 Commands
*
* See the datasheet for additional details:
* https://www.newhavendisplay.com/appnotes/datasheets/LCDs/ST7789V.pdf
*
**********************************************************************
*/
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

/*********************************************************************
*
*       LCD_X_InitController
*
* Purpose:
*   Initializes the display controller
*/
static void _InitController(void)
{
  /* Set up the display controller and put it into operation. If the
  *  display controller is not initialized by any external routine
  *  this needs to be adapted by the customer.
  */
	/* Reset the display controller */
		    SPI_st7735_write_reset_pin(0u);
		    GUI_Delay(100);
		    SPI_st7735_write_reset_pin(1u);
		    GUI_Delay(50);
		    Display_Write8_A0(ST7735_SWRESET);
		    GUI_Delay(200);
		    Display_Write8_A0(ST7735_SLPOUT);
		    GUI_Delay(255);
		    Display_Write8_A0(ST7735_FRMCTR1);
		    Display_Write8_A1(0x01);
		    Display_Write8_A1(0x2C);
		    Display_Write8_A1(0x2D);
		    Display_Write8_A0(ST7735_FRMCTR2);
		    Display_Write8_A1(0x01);
		    Display_Write8_A1(0x2C);
		    Display_Write8_A1(0x2D);
		    Display_Write8_A0(ST7735_FRMCTR3);
		    Display_Write8_A1(0x01);
		    Display_Write8_A1(0x2C);
		    Display_Write8_A1(0x2D);
		    Display_Write8_A1(0x01);
		    Display_Write8_A1(0x2C);
		    Display_Write8_A1(0x2D);
		    Display_Write8_A0(ST7735_INVCTR);
		    Display_Write8_A1(0x07);
		    Display_Write8_A0(ST7735_PWCTR1);
		    Display_Write8_A1(0xA2);
		    Display_Write8_A1(0x02);
		    Display_Write8_A1(0x84);
		    Display_Write8_A0(ST7735_PWCTR2);
		    Display_Write8_A1(0xC5);
		    Display_Write8_A0(ST7735_PWCTR3);
		    Display_Write8_A1(0x0A);
		    Display_Write8_A1(0x00);
		    Display_Write8_A0(ST7735_PWCTR4);
		    Display_Write8_A1(0x8A);
		    Display_Write8_A1(0x2A);
		    Display_Write8_A0(ST7735_PWCTR5);
		    Display_Write8_A1(0x8A);
		    Display_Write8_A1(0xEE);
		    Display_Write8_A0(ST7735_VMCTR1);
		    Display_Write8_A1(0x0E);
		    Display_Write8_A0(ST7735_INVON);
		    Display_Write8_A0(ST7735_MADCTL);
		    Display_Write8_A1(0xC8);
		    Display_Write8_A0(ST7735_COLMOD);
		    Display_Write8_A1(0x05);
		    Display_Write8_A0(ST7735_DISPON);
		    GUI_Delay(200);

}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/


void LCD_X_Config(void)
{
  //
  // Set display driver and color conversion for 1st layer
  //
  GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER, COLOR_CONVERSION, 0, 0);
  //
  // Display driver configuration
  //

  LCD_SetSizeEx (0, YSIZE_PHYS, XSIZE_PHYS);
  LCD_SetVSizeEx(0, YSIZE_PHYS, XSIZE_PHYS);
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task, the routine needs to be adapted to
*   the display controller. Note that the commands marked
*   "optional" are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Zero based layer index
*   Cmd        - Command to be executed
*   pData      - Pointer to a data structure.
*
* Return Value:
*   < -1 - Error
*     -1 - The command is not handled.
*      0 - OK.
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData)
{
  int r;

  GUI_USE_PARA(LayerIndex);
  GUI_USE_PARA(pData);

  switch (Cmd)
  {
  case LCD_X_INITCONTROLLER:
    //
    // Called during the initialization process in order to set up the
    // display controller and put it into operation. If the display
    // controller is not initialized by any external routine, this needs
    // to be adapted by the customer...
    //
    // ...
    _InitController();
    r = 0;
    break;

  default:
    r = -1;
    break;
  }

  return r;
}

#if defined(__cplusplus)
}
#endif


/*************************** End of file ****************************/
