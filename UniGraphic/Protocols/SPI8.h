#ifndef SPI8_H
#define SPI8_H

#include "mbed.h"
#include "Protocols.h"
//#include "GraphicsDisplay.h"

/** SPI 8bit interface
*/
class SPI8 : public Protocols
{
 public:

    /** Create an SPI 8bit display interface with 3 control pins
    *
    * @param SPI mosi
    * @param SPI miso
    * @param SPI sclk
    * @param CS pin connected to CS of display
    * @param reset pin connected to RESET of display
    * @param DC pin connected to data/command of display
    */ 
    SPI8(int Hz, PinName mosi, PinName miso, PinName sclk, PinName CS, PinName reset, PinName DC);
 
protected:
   
    /** Send 8bit command to display controller 
    *
    * @param cmd: byte to send  
    *
    */   
    virtual void wr_cmd8(unsigned char cmd);
    
    /** Send 8bit data to display controller 
    *
    * @param data: byte to send   
    *
    */   
    virtual void wr_data8(unsigned char data);
    
    /** Send 2x8bit command to display controller 
    *
    * @param cmd: halfword to send  
    */   
    virtual void wr_cmd16(unsigned short cmd);
    
    /** Send 2x8bit data to display controller 
    *
    * @param data: halfword to send   
    *
    */   
    virtual void wr_data16(unsigned short data);
    
    /** Send 16bit pixeldata to display controller 
    *
    * @param data: halfword to send   
    *
    */   
    virtual void wr_gram(unsigned short data);
    
    /** Send same 16bit pixeldata to display controller multiple times
    *
    * @param data: halfword to send
    * @param count: how many
    *
    */   
    virtual void wr_gram(unsigned short data, unsigned int count);
    
    /** Send array of pixeldata shorts to display controller
    *
    * @param data: unsigned short pixeldata array
    * @param lenght: lenght (in shorts)
    *
    */   
    virtual void wr_grambuf(unsigned short* data, unsigned int lenght);
    
    /** Read 16bit pixeldata from display controller (with dummy cycle)
    *
    * @returns 16bit color
    */ 
    virtual unsigned short rd_gram();
    
    /** Read 4x8bit register data (with dummy cycle)
    * @param reg the register to read
    * @returns data as uint
    * 
    */ 
    virtual unsigned int rd_reg_data32(unsigned char reg);
    
    /** Read 3x8bit ExtendedCommands register data
    * @param reg the register to read
    * @param SPIreadenablecmd vendor/device specific cmd to read EXTC registers
    * @returns data as uint
    * @note EXTC regs (0xB0 to 0xFF) are read/write registers but needs special cmd to be read in SPI mode
    */ 
    virtual unsigned int rd_extcreg_data32(unsigned char reg, unsigned char SPIreadenablecmd);
    
    /** HW reset sequence (without display init commands)   
    */
    virtual void hw_reset();
    
    /** Set ChipSelect high or low
    * @param enable 0/1   
    */
    virtual void BusEnable(bool enable);
  
    DigitalOut _CS;

private:

    SPI _spi;
    DigitalOut _reset;
    DigitalOut _DC;
  
};
#endif