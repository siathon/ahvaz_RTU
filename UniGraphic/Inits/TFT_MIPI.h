#ifndef MBED_TFT_MIPI_H
#define MBED_TFT_MIPI_H



#include "mbed.h"
#include "TFT.h"

/** Class for (quite)standard MIPI-DBI TypeB (parallel) and TypeC (spi) TFT display controller
* for Ilitek ILI9481, Renesas R61581, Raydium RM68042
* to be copypasted and adapted for other controllers
*/
class TFT_MIPI : public TFT
{
 
 public:

    /** Create a PAR display interface
    * @param displayproto only supports PAR_8
    * @param port GPIO port name to use
    * @param CS pin connected to CS of display
    * @param reset pin connected to RESET of display
    * @param DC pin connected to data/command of display
    * @param WR pin connected to SDI of display
    * @param RD pin connected to RS of display
    * @param name The name used by the parent class to access the interface
    * @param LCDSIZE_X x size in pixel - optional
    * @param LCDSIZE_Y y size in pixel - optional
    */ 
    TFT_MIPI(proto_t displayproto, PortName port, PinName CS, PinName reset, PinName DC, PinName WR, PinName RD, const char* name , unsigned int LCDSIZE_X = 320, unsigned  int LCDSIZE_Y = 480);
  
    /** Create an SPI display interface
    * @param displayproto only supports SPI_8
    * @param Hz SPI speed in Hz
    * @param mosi SPI pin
    * @param miso SPI pin
    * @param sclk SPI pin
    * @param CS pin connected to CS of display
    * @param reset pin connected to RESET of display
    * @param DC pin connected to data/command of display
    * @param name The name used by the parent class to access the interface
    * @param LCDSIZE_X x size in pixel - optional
    * @param LCDSIZE_Y y size in pixel - optional
    */ 
    TFT_MIPI(proto_t displayproto, int Hz, PinName mosi, PinName miso, PinName sclk, PinName CS, PinName reset, PinName DC, const char* name , unsigned int LCDSIZE_X = 320, unsigned  int LCDSIZE_Y = 480);
  

  
protected:
    
    
    /** Init command sequence  
    */
    void init();



};
#endif