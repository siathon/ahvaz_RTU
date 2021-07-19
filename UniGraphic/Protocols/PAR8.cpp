 /* mbed UniGraphic library - PAR8 protocol class
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
 
#include "PAR8.h"

PAR8::PAR8(PortName port, PinName CS, PinName reset, PinName DC, PinName WR, PinName RD)
    : _port(port,0xFF), _CS(CS), _reset(reset), _DC(DC), _WR(WR), _RD(RD)
{
    _reset = 1;
    _DC=1;
    _WR=1;
    _RD=1;
    _CS=1;
#ifdef STMPORTDEBUG
    findport(port); //on return, GPIO get disabled
#endif
    _port.mode(PullNone);
    _port.output(); // will re-enable our GPIO port
    hw_reset();    
}

#ifdef STMPORTDEBUG
// create a port obj with STM HAL drivers, just to collect memorymapped regs
void PAR8::findport(PortName port)
{
    port_t tempport;
    port_init(&tempport, port, 0xFF, PIN_INPUT);
    outreg = tempport.reg_out;
    inreg = tempport.reg_in;
 //   debug("out 0x%.8X  in 0x%.8X\r\n", outreg, inreg);
}
#endif
void PAR8::wr_cmd8(unsigned char cmd)
{   
#ifdef USE_CS
    _CS = 0;
#endif    
    _DC = 0; // 0=cmd
    _WR=0;
    _port.write(cmd);      // write 8bit
    _WR=1;
#ifdef USE_CS
    _CS = 1;
#endif
}
void PAR8::wr_data8(unsigned char data)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC = 1; // 1=data
    _WR=0;
    _port.write(data);    // write 8bit
    _WR=1;
#ifdef USE_CS
    _CS = 1;
#endif
}
void PAR8::wr_cmd16(unsigned short cmd)
{   
#ifdef USE_CS
    _CS = 0;
#endif    
    _DC = 0; // 0=cmd
    _WR=0;
    _port.write(cmd>>8);      // write 8bit
    _WR=1;
    _WR=0;
    _port.write(cmd&0xFF);      // write 8bit
    _WR=1;
#ifdef USE_CS
    _CS = 1;
#endif
}
void PAR8::wr_data16(unsigned short data)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC = 1; // 1=data
    _WR=0;
    _port.write(data>>8);    // write 8bit
    _WR=1;
    _WR=0;
    _port.write(data&0xFF);    // write 8bit
    _WR=1;
#ifdef USE_CS
    _CS = 1;
#endif
}
void PAR8::wr_gram(unsigned short data)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC = 1; // 1=data
    _WR=0;
    _port.write(data>>8);    // write 8bit
    _WR=1;
    _WR=0;
    _port.write(data&0xFF);    // write 8bit
    _WR=1;
#ifdef USE_CS
    _CS = 1;
#endif
}
void PAR8::wr_gram(unsigned short data, unsigned int count)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC = 1; // 1=data
    if((data>>8)==(data&0xFF))
    {
        count<<=1;
        _port.write(data);    // write 8bit
        while(count)
        {
            _WR=0;
            _WR=1;
            count--;
        }
    }
    else
    {
        while(count)
        {
            _WR=0;
            _port.write(data>>8);    // write 8bit
            _WR=1;
            _WR=0;
            _port.write(data&0xFF);    // write 8bit
            _WR=1;
            count--;
        }
    }
#ifdef USE_CS
    _CS = 1;
#endif
}
void PAR8::wr_grambuf(unsigned short* data, unsigned int lenght)
{
#ifdef USE_CS
    _CS = 0;
#endif
    _DC = 1; // 1=data
    while(lenght)
    {
        _WR=0;
        _port.write((*data)>>8);    // write 8bit
        _WR=1;
        _WR=0;
        _port.write((*data)&0xFF);    // write 8bit
        _WR=1;
        data++;
        lenght--;
    }
#ifdef USE_CS
    _CS = 1;
#endif
}
unsigned short PAR8::rd_gram()
{
#ifdef USE_CS
    _CS = 0;
#endif
    unsigned short r=0;
    _DC = 1; // 1=data
   _port.input();
   
    _RD = 0;
    _port.read(); //dummy read
    _RD = 1;
    
    _RD = 0;
//    _RD = 0; // add wait
    r |= (_port.read()&0xFF);
    r <<= 8;
    _RD = 1;
    
    _RD = 0;
//    _RD = 0; // add wait
    r |= (_port.read()&0xFF);
    _RD = 1;
    
#ifdef USE_CS
    _CS = 1;
#endif
    _port.output();
    return r;
}
unsigned int PAR8::rd_reg_data32(unsigned char reg)
{
#ifdef USE_CS
    _CS = 0;
#endif
    wr_cmd8(reg);
    unsigned int r=0;
    _DC = 1; // 1=data
   _port.input();
   
    _RD = 0;
    _port.read(); //dummy read
    _RD = 1;
    
    _RD = 0;
 //   _RD = 0; // add wait
    r |= (_port.read()&0xFF);
    r <<= 8;
    _RD = 1;
    
    _RD = 0;
 //   _RD = 0; // add wait
    r |= (_port.read()&0xFF);
    r <<= 8;
    _RD = 1;
    
    _RD = 0;
//    _RD = 0; // add wait
    r |= (_port.read()&0xFF);
    r <<= 8;
    _RD = 1;
    
    _RD = 0;
 //   _RD = 0; // add wait
    r |= (_port.read()&0xFF);
    _RD = 1;
    
    _CS = 1; // force CS HIG to interupt the cmd in case was not supported
#ifndef USE_CS //if CS is not used, force fixed LOW again
    _CS = 0;
#endif
    _port.output();
    return r;
}
// in Par mode EXTC regs (0xB0-0xFF) can be directly read
unsigned int PAR8::rd_extcreg_data32(unsigned char reg, unsigned char SPIreadenablecmd)
{
    return rd_reg_data32(reg);
}
void PAR8::hw_reset()
{
    wait_ms(15);
    _DC = 1;
    _CS = 1;
    _WR = 1;
    _RD = 1;
    _reset = 0;                        // display reset
    wait_us(50);
    _reset = 1;                       // end reset
    wait_ms(15);
#ifndef USE_CS
    _CS=0;      // put CS low now and forever
#endif
}
void PAR8::BusEnable(bool enable)
{
    _CS = enable ? 0:1;
}