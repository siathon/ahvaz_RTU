 /* mbed UniGraphic library - SPI8 protocol class
 * Copyright (c) 2015 Giuliano Dianda
 * Released under the MIT License: http://mbed.org/license/mit
 *
 * Derived work of:
 *
 * mbed library for 240*320 pixel display TFT based on ILI9341 LCD Controller
 * Copyright (c) 2013 Peter Drescher - DC2PD
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#include "SPI8.h"

//#define USE_CS

SPI8::SPI8(int Hz, PinName mosi, PinName miso, PinName sclk, PinName CS, PinName reset, PinName DC)
    : _CS(CS), _spi(mosi, miso, sclk), _reset(reset), _DC(DC)
{
    _reset = 1;
    _DC=1;
    _CS=1;
    _spi.format(8,0);                  // 8 bit spi mode 0
 //   _spi.frequency(12000000);          // 10 Mhz SPI clock, 12mhz for F411
    _spi.frequency(Hz);
    hw_reset();    
}

void SPI8::wr_cmd8(unsigned char cmd)
{   
#ifdef USE_CS
    _CS = 0;
#endif    
    _DC.write(0); // 0=cmd
    _spi.write(cmd);      // write 8bit
#ifdef USE_CS
    _CS = 1;
#endif
}
void SPI8::wr_data8(unsigned char data)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC.write(1); // 1=data
    _spi.write(data);    // write 8bit
#ifdef USE_CS
    _CS = 1;
#endif
}
void SPI8::wr_cmd16(unsigned short cmd)
{   
#ifdef USE_CS
    _CS = 0;
#endif    
    _DC.write(0); // 0=cmd
    _spi.write(cmd>>8);      // write 8bit
    _spi.write(cmd&0xFF);      // write 8bit
#ifdef USE_CS
    _CS = 1;
#endif
}
void SPI8::wr_data16(unsigned short data)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC.write(1); // 1=data
    _spi.write(data>>8);    // write 8bit
    _spi.write(data&0xFF);    // write 8bit
#ifdef USE_CS
    _CS = 1;
#endif
}
void SPI8::wr_gram(unsigned short data)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC.write(1); // 1=data
    _spi.write(data>>8);    // write 8bit
    _spi.write(data&0xFF);    // write 8bit
#ifdef USE_CS
    _CS = 1;
#endif
}
void SPI8::wr_gram(unsigned short data, unsigned int count)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC.write(1); // 1=data
    if((data>>8)==(data&0xFF))
    {
        count<<=1;
        while(count)
        {
            _spi.write(data);    // write 8bit
            count--;
        }
    }
    else
    {
        while(count)
        {
            _spi.write(data>>8);    // write 8bit
            _spi.write(data&0xFF);    // write 8bit
            count--;
        }
    }
#ifdef USE_CS
    _CS = 1;
#endif
}
void SPI8::wr_grambuf(unsigned short* data, unsigned int lenght)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC.write(1); // 1=data
    while(lenght)
    {
        _spi.write((*data)>>8);    // write 8bit
        _spi.write((*data)&0xFF);    // write 8bit
        data++;
        lenght--;
    }
#ifdef USE_CS
    _CS = 1;
#endif
}
unsigned short SPI8::rd_gram()
{
#ifdef USE_CS
    _CS = 0;
#endif
    unsigned int r=0;
    _DC.write(1); // 1=data
    _spi.write(0); // whole first byte is dummy
    r |= _spi.write(0);
    r <<= 8;
    r |= _spi.write(0);
    r <<= 8;
    r |= _spi.write(0);  
    _CS = 1; // force CS HIG to interupt the "read state"
#ifndef USE_CS //if CS is not used, force fixed LOW again
    _CS = 0;
#endif
    // gram is 18bit/pixel, if you set 16bit/pixel (cmd 3A), during writing the 16bits are expanded to 18bit
    // during reading, you read the raw 18bit gram
    r = RGB18to16((r&0xFC0000)>>16, (r&0xFC00)>>8, r&0xFC);// 18bit pixel, rrrrrr00_gggggg00_bbbbbb00, converted to 16bit
    return (unsigned short)r;
}
unsigned int SPI8::rd_reg_data32(unsigned char reg)
{
#ifdef USE_CS
    _CS = 0;
#endif
    wr_cmd8(reg);
    unsigned int r=0;
    _DC.write(1); // 1=data
   
    r |= _spi.write(0); // we get only 7bit valid, first bit was the dummy cycle
    r <<= 8;
    r |= _spi.write(0);
    r <<= 8;
    r |= _spi.write(0);
    r <<= 8;
    r |= _spi.write(0);
    r <<= 1; // 32bits are aligned, now collecting bit_0
    r |= (_spi.write(0) >> 7);
    // we clocked 7 more bit so ILI waiting for 8th, we need to reset spi bus
    _CS = 1; // force CS HIG to interupt the cmd
#ifndef USE_CS //if CS is not used, force fixed LOW again
    _CS = 0;
#endif
    return r;
}
unsigned int SPI8::rd_extcreg_data32(unsigned char reg, unsigned char SPIreadenablecmd)
{
    unsigned int r=0;
    for(int regparam=1; regparam<4; regparam++) // when reading EXTC regs, first parameter is always dummy, so start with 1
    {
        wr_cmd8(SPIreadenablecmd);  // spi-in enable cmd, 0xD9 (ili9341) or 0xFB (ili9488) or don't know
        wr_data8(0xF0|regparam);    // in low nibble specify which reg parameter we want
        wr_cmd8(reg);               // now send cmd (select register we want to read)
        _DC.write(1); // 1=data
        r <<= 8;
        r |= _spi.write(0);
        // r = _spi.write(0) >> 8; for 16bit
    }
_CS = 1; // force CS HIG to interupt the cmd
#ifndef USE_CS //if CS is not used, force fixed LOW again
    _CS = 0;
#endif
    return r;
}
void SPI8::hw_reset()
{
    wait_ms(15);
    _DC = 1;
 //   _CS = 1;
    _CS = 0;
    _reset = 0;                        // display reset
    wait_us(50);
    _reset = 1;                       // end reset
    wait_ms(15);
#ifndef USE_CS
    _CS=0;      // put CS low now and forever
#endif
}
void SPI8::BusEnable(bool enable)
{
    _CS = enable ? 0:1;
}