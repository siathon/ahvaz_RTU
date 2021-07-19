 /* mbed UniGraphic library - Abstract protocol class
 * Copyright (c) 2015 Giuliano Dianda
 * Released under the MIT License: http://mbed.org/license/mit
 */
 
/** @file Protocols.h
*/
#ifndef Protocols_H
#define Protocols_H

#include "mbed.h"

#define RGB18to16(r,g,b)  (((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3)) //5 red | 6 green | 5 blue
#define BGR2RGB(color) (((color&0x1F)<<11) | (color&0x7E0) | ((color&0xF800)>>11))

//#define USE_CS

/** Protocol types
*/
enum proto_t {
    PAR_8   /**< Parallel 8bit, pins 0 to 7 */
    ,PAR_16 /**< Parallel 16bit, pins 0 to 15 */
    ,SPI_8  /**< SPI 8bit */
    ,SPI_16 /**< SPI 16bit */
};


/** Abstract interface class for spi and parallel protocols
*/
class Protocols
{
 public:
    
    /** Send 8bit command to display controller 
    *
    * @param cmd: byte to send  
    *
    */   
    virtual void wr_cmd8(unsigned char cmd) = 0;
    
    /** Send 8bit data to display controller 
    *
    * @param data: byte to send   
    *
    */   
    virtual void wr_data8(unsigned char data) = 0;
    
    /** Send 2x8bit command to display controller 
    *
    * @param cmd: halfword to send  
    *
    */   
    virtual void wr_cmd16(unsigned short cmd) = 0;
    
    /** Send 2x8bit data to display controller 
    *
    * @param data: halfword to send   
    *
    */   
    virtual void wr_data16(unsigned short data) = 0;
    
    /** Send 16bit pixeldata to display controller 
    *
    * @param data: halfword to send   
    *
    */   
    virtual void wr_gram(unsigned short data) = 0;
    
    /** Send same 16bit pixeldata to display controller multiple times
    *
    * @param data: halfword to send
    * @param count: how many
    *
    */   
    virtual void wr_gram(unsigned short data, unsigned int count) = 0;
    
    /** Send array of pixeldata shorts to display controller
    *
    * @param data: unsigned short pixeldata array
    * @param lenght: lenght (in shorts)
    *
    */   
    virtual void wr_grambuf(unsigned short* data, unsigned int lenght) = 0;
    
    /** Read 16bit pixeldata from display controller (with dummy cycle)
    *
    * @returns 16bit color
    */ 
    virtual unsigned short rd_gram() = 0;
    
    /** Read 4x8bit register data (with dummy cycle)
    * @param reg the register to read
    * @returns data as uint
    * 
    */ 
    virtual unsigned int rd_reg_data32(unsigned char reg) = 0;
    
    /** Read 3x8bit ExtendedCommands register data
    * @param reg the register to read
    * @param SPIreadenablecmd vendor/device specific cmd to read EXTC registers
    * @returns data as uint
    * @note EXTC regs (0xB0 to 0xFF) are read/write registers but needs special cmd to be read in SPI mode
    */ 
    virtual unsigned int rd_extcreg_data32(unsigned char reg, unsigned char SPIreadenablecmd) = 0;
    
    /** HW reset sequence (without display init commands)   
    */
    virtual void hw_reset() = 0;
    
    /** Set ChipSelect high or low
    * @param enable 0/1   
    */
    virtual void BusEnable(bool enable) = 0;

};
#endif