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

#ifndef RFSIMPLICITI_H_
#define RFSIMPLICITI_H_

// *************************************************************************************************
// Include section

// *************************************************************************************************
// Prototypes section
extern void reset_rf(void);

extern void sx_rf2chronos(u8 line);

//extern void sx_rf2chronos_link(u8 line);
extern void display_rf2chronos(u8 line, u8 update);

extern void send_smpl_data(u16 data);
extern u8 is_rf(void);
extern u8 set_simpliciti_ed_address(void);

extern void update_simpliciti_timer(void);
extern void start_simpliciti_sync_timer(void);
extern void start_simpliciti_2chronos(void);
extern void change_link_listen(void);
extern void stop_simpliciti_sync_timer(void);

// *************************************************************************************************
// Defines section

// SimpliciTI connection states
typedef enum
{
    SIMPLICITI_OFF = 0,         // Not connected
    SIMPLICITI_ACCELERATION,    // Transmitting acceleration data and button events
    SIMPLICITI_BUTTONS,         // Transmitting button events
    SIMPLICITI_SYNC             // Syncing
} simpliciti_mode_t;

// Stop SimpliciTI transmission after 60 minutes to save power
#define SIMPLICITI_TIMEOUT                          (60 * 60u)

// Timer interrupt for 2 chronos communication
#define SIMPLICITI_2CHRONOS_TICK                (32768 / 2)

// Interval of transmission for communication between two chronos
#define SIMPLICITI_2CHRONOS_INTERVAL    (1u)

// Button flags for SimpliciTI data
#define SIMPLICITI_BUTTON_STAR                  (0x10)
#define SIMPLICITI_BUTTON_NUM                   (0x20)
#define SIMPLICITI_BUTTON_UP                    (0x30)

// SimpliciTI mode flag
#define SIMPLICITI_MOUSE_EVENTS                 (0x01)
#define SIMPLICITI_KEY_EVENTS                   (0x02)

// *************************************************************************************************
// Global Variable section
struct RFsmpl
{
    // SIMPLICITI_OFF, SIMPLICITI_ACCELERATION, SIMPLICITI_BUTTONS
    simpliciti_mode_t mode;

    // Timeout until SimpliciTI transmission is automatically stopped
    u16 timeout;

    // Variable to display
    u8 display_sync_done;
};
extern struct RFsmpl sRFsmpl;

extern unsigned char simpliciti_flag;

#define SIMPLICITI_2CHRONOS_MISS_COUNT  (3u)
#define SIMPLICITI_2CHRONOS_TIMEOUT         (15u)

struct RF2chronos
{
    // Variable to display in this watch
    u8 c2c_value;

    // Variable to display in the other watch
    u8 c2c_send_value;

    // Contains the information of what button was pressed. This will be send to the other watch
    u8 RF_2chronos_button;

    // Used to synchronize the timers from both watches
    u8 RF_2chronos_timer_offset;

    // Timeout for connection
    u8 timeout;

    // It is set to zero every time the device receives a message
    // It is incremented once a second
    // If the watch doesn't receive a
    u8 error_in_messages;
};
extern struct RF2chronos sRF2chronos;

extern void oslo_conference(void);

// *************************************************************************************************
// Extern section

#endif                          /*RFSIMPLICITI_H_ */
