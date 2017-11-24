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
// Dietmar Schneider, MSP430 Tools, 12/22/2010

#ifndef POWERMETER_H
#define POWERMETER_H

// *************************************************************************************************
// Include section
#include <project.h>

// *************************************************************************************************
// Prototypes section
extern void start_powermeter(void);
extern void stop_powermeter(void);
extern void reset_powermeter(void);
extern u8 is_powermeter(void);
extern void mx_powermeter(u8 line);
extern void sx_powermeter(u8 line);
extern void display_powermeter(u8 line, u8 update);
extern void display_rssi_value(void);
extern void powermeter_measurement(void);
extern void reset_powermeter(void);

// *************************************************************************************************
// Defines section
#define POWERMETER_UNINITIALIZED                 (0u)
#define POWERMETER_STOP                          (1u)
#define POWERMETER_RUN                           (2u)


// *************************************************************************************************
// Global Variable section
struct powermeter
{
    u8 state;
    u8 drawFlag;
    signed char rssi;
};
extern struct powermeter sPowermeter;

// *************************************************************************************************
// Extern section

#endif              /*POWERMETER_H */
