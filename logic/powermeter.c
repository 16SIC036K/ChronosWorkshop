// *************************************************************************************************
//
//      Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
//
//
//        Redistribution and use in source and binary forms, with or without
//        modification, are permitted provided that the following conditions
//        are met:
//
//          Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//
//          Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the
//          distribution.
//
//          Neither the name of Texas Instruments Incorporated nor the names of
//          its contributors may be used to endorse or promote products derived
//          from this software without specific prior written permission.
//
//        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//        A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//        OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//        SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//        LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//        DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//        THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//        OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
// Powermeter functions.
// Dietmar Schneider, MSP430, Dec 2010
// *************************************************************************************************

// *************************************************************************************************
// Include section

// system
#include "project.h"
#include <string.h>

// driver
#include "powermeter.h"
#include "ports.h"
#include "display.h"
#include "timer.h"
#include "rf1a.h"
#include "pmm.h"

// logic
#include "menu.h"

// *************************************************************************************************
// Prototypes section
void start_powermeter(void);
void stop_powermeter(void);
void mx_powermeter(u8 line);
void sx_powermeter(u8 line);
void display_powermeter(u8 line, u8 update);
u8 is_powermeter(void);
void display_rssi_value(void);
void powermeter_measurement(void);
void reset_powermeter(void);

// *************************************************************************************************
// Defines section

// *************************************************************************************************
// Global Variable section
struct powermeter sPowermeter;

// *************************************************************************************************
// Extern section

// *************************************************************************************************
// @fn          is_powermeter
// @brief       Is powermeter operating and visible?
// @param       none
// @return      1=powermeter_RUN, 0=other states
// *************************************************************************************************
u8 is_powermeter(void)
{
    if ((sPowermeter.state == POWERMETER_RUN) && (ptrMenu_L1 != &menu_L1_Powermeter)){
        stop_powermeter();
    }
    return (sPowermeter.state == POWERMETER_RUN);
}

void reset_powermeter(void){
    sPowermeter.state = POWERMETER_STOP;
    request.flag.powermeter_measurement = 0;
    display.flag.update_powermeter = 0;
}

void powermeter_measurement(void){
    if (sPowermeter.state == POWERMETER_RUN){
        volatile unsigned char rssi;
        volatile unsigned char rf_status;
        volatile unsigned long counter = 0;

        Strobe(RF_SFRX);

        //Write f as FREQ[0..23] register value
        // 0x222762 ~ 888 MHz
        WriteSingleReg(FREQ2, 0x22);     // FREQ2
        WriteSingleReg(FREQ1, 0x27);     // FREQ1
        WriteSingleReg(FREQ0, 0x62);     // FREQ0

        RF1AIES &= ~BIT1;
        RF1AIES &= ~BIT2;
        RF1AIFG &= ~BIT1;                // clear GDO1 bit
        RF1AIFG &= ~BIT2;                //clear PLL_LOCK ISR

        rf_status = Strobe(RF_SRX);      //performs calibration first (see MCSM0 reg.)

        while ((RF1AIFG & BIT2) == 0) ;  // Wait for PLL lock

        while (!(RF1AIFG & BIT1)) ;      // wait for RSSI valid on GDO1

        counter = 0;
        rssi = ReadSingleReg(RSSI);
        while (rssi == ReadSingleReg(RSSI)){
            rssi = ReadSingleReg(RSSI);
            // Escape sequence if rssi frozen or really stable.
            if (counter++ == 1000) break;
        }
        counter--;                       // counter measured:
        sPowermeter.rssi = rssi;
        Strobe(RF_SIDLE);
        display.flag.update_powermeter = 1;

    }
}

// *************************************************************************************************
// @fn          start_powermeter
// @brief       Starts powermeter timer interrupt and sets powermeter state to on.
// @param       none
// @return      none
// *************************************************************************************************
void start_powermeter(void)
{
    // Set Powermeter run flag
    sPowermeter.state = POWERMETER_RUN;

    Strobe(RF_SRES);                          // Reset the Radio Core
    Strobe(RF_SNOP);                          // Reset Radio Pointer
    Strobe(RF_SIDLE);

    //Map GDO2 to 0x0A PLL_LOCK .. positive transition = in lock. Also check if FSCAL1 != 0x3f
    WriteSingleReg(IOCFG2, 0x0a);
    //Map GDO1 to  0x1e RSSI_Valid in IOCFG1 (0x02)
    WriteSingleReg(IOCFG1, 0x1e);

    //Set Packet Lenght to 59 .. increase robustnes. unlikely that it exaclty fits size.
    WriteSingleReg(PKTLEN, 59);
    //Enable PKTCTRL w/ max pqt, adresscheck and crc autoflush
    WriteSingleReg(PKTCTRL1, BIT7 + BIT6 + BIT5 + BIT3 + BIT0);
    WriteSingleReg(PKTCTRL0, BIT2);
    WriteSingleReg(ADDR, 59);

    // SET IF
    WriteSingleReg(FSCTRL1, 15);     // 381 kHz
    // Set RBW and Datarate !!
    WriteSingleReg(MDMCFG4, 0x8c);   // 203 kHz RBW and DRATE_E = 0xC
    WriteSingleReg(MDMCFG3, 0x22);   // DRATE_M = 0x22 -> 115 kBd

    //Set synch mode to 30/32 + carrier-sense above threshold
    WriteSingleReg(MDMCFG2, 7);
    // 24 bytes preamble
    WriteSingleReg(MDMCFG1, 0x72);

    // Stay in Rx after packet received
    WriteSingleReg(MCSM1, BIT2 + BIT3);
    // Autocal on When going from IDLE to RX or TX (or FSTXON)
    WriteSingleReg(MCSM0, BIT4);

    // Disable Freq Offset Comp and Data Rate offset Comp
    WriteSingleReg(FOCCFG, 0);
    WriteSingleReg(BSCFG, 0);

    //CHANBW_M 01, _E 00 -> 600 kHz Rx filter
    WriteSingleReg(MDMCFG4, 0x0c + BIT4);

}

// *************************************************************************************************
// @fn          stop_powermeter
// @brief       Stops powermeter timer interrupt and sets powermeter state to off.
//                              Does not reset powermeter count.
// @param       none
// @return      none
// *************************************************************************************************
void stop_powermeter(void)
{
    // Clear Powermeter run flag
    sPowermeter.state = POWERMETER_STOP;
    Strobe(RF_SPWD);

    // Clear RF icon
    display_symbol(LCD_ICON_BEEPER1, SEG_OFF);
    display_symbol(LCD_ICON_BEEPER2, SEG_OFF);
    display_symbol(LCD_ICON_BEEPER3, SEG_OFF);

    // Call draw routine immediately
    display_powermeter(LINE1, DISPLAY_LINE_UPDATE_FULL);

}

// *************************************************************************************************
// @fn          mx_powermeter
// @brief       powermeter set routine. Mx stops powermeter.
// @param       u8 line LINE2
// @return      none
// *************************************************************************************************
void mx_powermeter(u8 line)
{
    // Stop Powermeter
    stop_powermeter();

    // Display RSSI
    display_powermeter(line, DISPLAY_LINE_UPDATE_FULL);
}

// *************************************************************************************************
// @fn          sx_powermeter
// @brief       powermeter direct function. Button DOWN starts/stops powermeter, but does not reset
// count.
// @param       u8 line LINE2
// @return      none
// *************************************************************************************************
void sx_powermeter(u8 line)
{
    // Up: RUN, STOP
    if (button.flag.up)
    {
        if (sPowermeter.state == POWERMETER_STOP)
        {
            // Start Powermeter
            start_powermeter();
        }
        else
        {
            // Stop Powermeter
            stop_powermeter();
        }

    }
}

// *************************************************************************************************
// @fn          display_powermeter
// @brief       powermeter user routine. Sx starts/stops powermeter.
// @param       u8 line LINE2
//              u8 update       DISPLAY_LINE_CLEAR
// @return      none
// *************************************************************************************************
void display_powermeter(u8 line, u8 update)
{
    if (line == LINE1){
        if (update == DISPLAY_LINE_CLEAR)
        {
            // Stop Powermeter
            stop_powermeter();

            // Clean up symbols when leaving function
            clear_line(LINE1);
            // Clear RF icon - clear from powermeter
            display_symbol(LCD_ICON_BEEPER1, SEG_OFF);
            display_symbol(LCD_ICON_BEEPER2, SEG_OFF);
            display_symbol(LCD_ICON_BEEPER3, SEG_OFF);
        }
        else
        {
            if (sPowermeter.state == POWERMETER_RUN)
            {
                display_rssi_value();
            }else
                display_chars(LCD_SEG_L1_3_0, (u8 *) "RSSI", SEG_ON);
        }
    }
}

void blinkRfSymbol(void){
    static unsigned char counter = 0;

    switch (counter){
        case 0: display_symbol(LCD_ICON_BEEPER1, SEG_OFF);
            display_symbol(LCD_ICON_BEEPER2, SEG_OFF);
            display_symbol(LCD_ICON_BEEPER3, SEG_OFF);
            break;
        case 1: display_symbol(LCD_ICON_BEEPER1, SEG_ON);
            break;
        case 2:         //display_symbol(LCD_ICON_BEEPER1, SEG_OFF);
            display_symbol(LCD_ICON_BEEPER2, SEG_ON);
            break;
        case 3:         //display_symbol(LCD_ICON_BEEPER2, SEG_OFF);
            display_symbol(LCD_ICON_BEEPER3, SEG_ON);
            break;
        default: break;
    }
    if (++counter > 3) counter = 0;
}

void display_rssi_value(void){
    u8 *str;
    signed char dBm = sPowermeter.rssi / 2 - 74;

    if (dBm < 0){
        dBm *= (-1);
        str = int_to_array((u32)dBm, 4, 0);
        str[0] = '-';
    }else{
        str = int_to_array((u32)dBm, 4, 0);
        str[0] = ' ';
    }
    if (dBm < 100) str[1] = ' ';
    display_chars(LCD_SEG_L1_3_0, str, SEG_ON);
    blinkRfSymbol();
}

