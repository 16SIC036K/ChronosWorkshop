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
// SimpliciTI functions.
// *************************************************************************************************

// *************************************************************************************************
// Include section

// system
#include "project.h"

// driver
#include "display.h"
#include "vti_as.h"
#include "ports.h"
#include "timer.h"
#include "radio.h"
#include "buzzer.h"

// logic
#include "acceleration.h"
#include "rfsimpliciti.h"
#include "bluerobin.h"
#include "simpliciti.h"
#include "clock.h"
#include "date.h"
#include "alarm.h"
#include "temperature.h"
#include "menu.h"

#include "..\simpliciti\Components\mrfi\smartrf\CC430\smartrf_CC430.h"
// *************************************************************************************************
// Prototypes section
void simpliciti_get_data_callback(void);
void start_simpliciti_sync(void);


// *************************************************************************************************
// Defines section

// Each packet index requires 2 bytes, so we can have 9 packet indizes in 18 bytes usable payload
#define BM_SYNC_BURST_PACKETS_IN_DATA           (9u)

// *************************************************************************************************
// Global Variable section
struct RFsmpl sRFsmpl;

// flag contains status information, trigger to send data and trigger to exit SimpliciTI
unsigned char simpliciti_flag;

// 4 data bytes to send
unsigned char simpliciti_data[SIMPLICITI_MAX_PAYLOAD_LENGTH];

// 4 byte device address overrides SimpliciTI end device address set in "smpl_config.dat"
unsigned char simpliciti_ed_address[4];

// Length of data
unsigned char simpliciti_payload_length;

// 1 = send one or more reply packets, 0 = no need to reply
//unsigned char simpliciti_reply;
unsigned char simpliciti_reply_count;

// 1 = send packets sequentially from burst_start to burst_end, 2 = send packets addressed by their
// index
u8 burst_mode;

// Start and end index of packets to send out
u16 burst_start, burst_end;

// Array containing requested packets
u16 burst_packet[BM_SYNC_BURST_PACKETS_IN_DATA];

// Current packet index
u8 burst_packet_index;

struct RF2chronos sRF2chronos;
// *************************************************************************************************
// Extern section
extern void (*fptr_lcd_function_line1)(u8 line, u8 update);
extern void (*fptr_lcd_function_line2)(u8 line, u8 update);
// *************************************************************************************************
// @fn          reset_rf
// @brief       Reset SimpliciTI data.
// @param       none
// @return      none
// *************************************************************************************************
void reset_rf(void)
{
    // No connection
    sRFsmpl.mode = SIMPLICITI_OFF;

    // Standard packets are 4 bytes long
    simpliciti_payload_length = 4;
}

// *************************************************************************************************
// @fn          sx_rf2chronos
// @brief       Start SimpliciTI. Button DOWN connects/disconnects to other eZ430-Chronos
// @param       u8 line         LINE2
// @return      none
// *************************************************************************************************
void sx_rf2chronos(u8 line)
{
    // Exit if battery voltage is too low for radio operation
    if (sys.flag.low_battery)
        return;

    // Exit if BlueRobin stack is active
    if (is_bluerobin())
        return;

    // Turn off the backlight
    P2OUT &= ~BUTTON_BACKLIGHT_PIN;
    P2DIR &= ~BUTTON_BACKLIGHT_PIN;

    BUTTONS_IE &= ~BUTTON_BACKLIGHT_PIN;

    // Start SimpliciTI in sync mode
    //start_simpliciti_2chronos_listen();

    start_simpliciti_2chronos();

    BUTTONS_IE |= BUTTON_BACKLIGHT_PIN;
}

// *************************************************************************************************
// @fn          display_rf2Chronos
// @brief       SimpliciTI display routine.
// @param       u8 line                 LINE2
//                              u8 update               DISPLAY_LINE_UPDATE_FULL
// @return      none
// *************************************************************************************************
void display_rf2chronos(u8 line, u8 update)
{
    if (update == DISPLAY_LINE_UPDATE_FULL)
    {
        display_chars(LCD_SEG_L2_5_0, (u8 *) "  C2C ", SEG_ON);
    }
}

// *************************************************************************************************
// @fn          is_rf
// @brief       Returns TRUE if SimpliciTI receiver is connected.
// @param       none
// @return      u8
// *************************************************************************************************
u8 is_rf(void)
{
    return (sRFsmpl.mode != SIMPLICITI_OFF);
}

// *************************************************************************************************
// @fn          simpliciti_get_ed_data_callback
// @brief       Callback function to read end device data from acceleration sensor (if available)
//                              and trigger sending. Can be also be used to transmit other data at
// different packet rates.
//                              Please observe the applicable duty limit in the chosen ISM band.
// @param       none
// @return      none
// *************************************************************************************************
void simpliciti_get_ed_data_callback(void)
{
    static u8 packet_counter = 0;

    if (sRFsmpl.mode == SIMPLICITI_ACCELERATION)
    {
        // Wait for next sample
        Timer0_A4_Delay(CONV_MS_TO_TICKS(5));

        // Read from sensor if DRDY pin indicates new data (set in PORT2 ISR)
        if (request.flag.acceleration_measurement && ((AS_INT_IN & AS_INT_PIN) == AS_INT_PIN))
        {
            // Clear flag
            request.flag.acceleration_measurement = 0;

            // Get data from sensor
            as_get_data(sAccel.xyz);

            // Transmit only every 3rd data set (= 33 packets / second)
            if (packet_counter++ > 1)
            {
                // Reset counter
                packet_counter = 0;

                // Store XYZ data in SimpliciTI variable
                simpliciti_data[1] = sAccel.xyz[0];
                simpliciti_data[2] = sAccel.xyz[1];
                simpliciti_data[3] = sAccel.xyz[2];

                // Trigger packet sending
                simpliciti_flag |= SIMPLICITI_TRIGGER_SEND_DATA;
            }
        }
    }
    else // transmit only button events
    {
        // New button event is stored in data
        if ((packet_counter == 0) && (simpliciti_data[0] & 0xF0) != 0)
        {
            packet_counter = 5;
        }

        // Send packet several times
        if (packet_counter > 0)
        {
            // Clear button event when sending last packet
            if (--packet_counter == 0)
            {
                simpliciti_data[0] &= ~0xF0;
            }
            else
            {
                // Trigger packet sending in regular intervals
                Timer0_A4_Delay(CONV_MS_TO_TICKS(30));
                simpliciti_flag |= SIMPLICITI_TRIGGER_SEND_DATA;
            }
        }
        else
        {
            // Wait in LPM3 for next button press
            _BIS_SR(LPM3_bits + GIE);
            __no_operation();
        }
    }

    // Update clock every 1/1 second
    if (display.flag.update_time)
    {
        display_time(LINE1, DISPLAY_LINE_UPDATE_PARTIAL);
        display.flag.update_time = 0;

        // Service watchdog
        WDTCTL = WDTPW + WDTIS__512K + WDTSSEL__ACLK + WDTCNTCL;
    }
}

// *************************************************************************************************
// @fn          start_simpliciti_sync
// @brief       Start SimpliciTI (sync mode).
// @param       none
// @return      none
// *************************************************************************************************
void start_simpliciti_sync(void)
{
    // Clear LINE1
    //   clear_line(LINE1);
    //   fptr_lcd_function_line1(LINE1, DISPLAY_LINE_CLEAR);

    // Stop acceleration sensor
    as_stop();

    // Get updated altitude
    //start_altitude_measurement();
    //stop_altitude_measurement();

    // Get updated temperature
    temperature_measurement(FILTER_OFF);

    // Turn on beeper icon to show activity
    display_symbol(LCD_ICON_BEEPER1, SEG_ON_BLINK_ON);
    display_symbol(LCD_ICON_BEEPER2, SEG_ON_BLINK_ON);
    display_symbol(LCD_ICON_BEEPER3, SEG_ON_BLINK_ON);

    // Debounce button event
    Timer0_A4_Delay(CONV_MS_TO_TICKS(BUTTONS_DEBOUNCE_TIME_OUT));

    // Prepare radio for RF communication
    open_radio();

    // Set SimpliciTI mode
    sRFsmpl.mode = SIMPLICITI_SYNC;

    // Set SimpliciTI timeout to save battery power
    sRFsmpl.timeout = SIMPLICITI_TIMEOUT;

    // Start SimpliciTI stack. Try to link to access point.
    // Exit with timeout or by a button DOWN press.
    if (simpliciti_link())
    {
        // Enter sync routine. This will send ready-to-receive packets at regular intervals to the
        // access point.
        // The access point replies with a command (NOP if no other command is set)
        simpliciti_main_sync();
    }

    // Set SimpliciTI state to OFF
    sRFsmpl.mode = SIMPLICITI_OFF;

    // Powerdown radio
    close_radio();

    // Clear last button events
    Timer0_A4_Delay(CONV_MS_TO_TICKS(BUTTONS_DEBOUNCE_TIME_OUT));
    BUTTONS_IFG = 0x00;
    button.all_flags = 0;

    // Clear icons
    display_symbol(LCD_ICON_BEEPER1, SEG_OFF_BLINK_OFF);
    display_symbol(LCD_ICON_BEEPER2, SEG_OFF_BLINK_OFF);
    display_symbol(LCD_ICON_BEEPER3, SEG_OFF_BLINK_OFF);

    // Force full display update
    display.flag.full_update = 1;
}

// *************************************************************************************************
// @fn          simpliciti_sync_decode_ap_cmd_callback
// @brief       For SYNC mode only: Decode command from access point and trigger actions.
// @param       none
// @return      none
// *************************************************************************************************
void simpliciti_sync_decode_ap_cmd_callback(void)
{
    u8 i;
    s16 t1, offset;

    // Default behaviour is to send no reply packets
    simpliciti_reply_count = 0;

    switch (simpliciti_data[0])
    {
        case SYNC_AP_CMD_NOP:
            break;

        case SYNC_AP_CMD_GET_STATUS: // Send watch parameters
            simpliciti_data[0] = SYNC_ED_TYPE_STATUS;
            // Send single reply packet
            simpliciti_reply_count = 1;
            break;

        case SYNC_AP_CMD_SET_WATCH:  // Set watch parameters
            sys.flag.use_metric_units = (simpliciti_data[1] >> 7) & 0x01;
            sTime.hour = simpliciti_data[1] & 0x7F;
            sTime.minute = simpliciti_data[2];
            sTime.second = simpliciti_data[3];
            sDate.year = (simpliciti_data[4] << 8) + simpliciti_data[5];
            sDate.month = simpliciti_data[6];
            sDate.day = simpliciti_data[7];
            sAlarm.hour = simpliciti_data[8];
            sAlarm.minute = simpliciti_data[9];
            // Set temperature and temperature offset
            t1 = (s16) ((simpliciti_data[10] << 8) + simpliciti_data[11]);
            offset = t1 - (sTemp.degrees - sTemp.offset);
            sTemp.offset = offset;
            sTemp.degrees = t1;
            // Set altitude
            //      sAlt.altitude = (s16) ((simpliciti_data[12] << 8) + simpliciti_data[13]);
            //      update_pressure_table(sAlt.altitude, sAlt.pressure, sAlt.temperature);

            display_chars(LCD_SEG_L2_5_0, (u8 *) "  DONE", SEG_ON);
            sRFsmpl.display_sync_done = 1;
            break;

        case SYNC_AP_CMD_GET_MEMORY_BLOCKS_MODE_1:
            // Send sequential packets out in a burst
            simpliciti_data[0] = SYNC_ED_TYPE_MEMORY;
            // Get burst start and end packet
            burst_start = (simpliciti_data[1] << 8) + simpliciti_data[2];
            burst_end = (simpliciti_data[3] << 8) + simpliciti_data[4];
            // Set burst mode
            burst_mode = 1;
            // Number of packets to send
            simpliciti_reply_count = burst_end - burst_start;
            break;

        case SYNC_AP_CMD_GET_MEMORY_BLOCKS_MODE_2:
            // Send specified packets out in a burst
            simpliciti_data[0] = SYNC_ED_TYPE_MEMORY;
            // Store the requested packets
            for (i = 0; i < BM_SYNC_BURST_PACKETS_IN_DATA; i++)
            {
                burst_packet[i] = (simpliciti_data[i * 2 + 1] << 8) + simpliciti_data[i * 2 + 2];
            }
            // Set burst mode
            burst_mode = 2;
            // Number of packets to send
            simpliciti_reply_count = BM_SYNC_BURST_PACKETS_IN_DATA;
            break;

        case SYNC_AP_CMD_ERASE_MEMORY: // Erase data logger memory
            break;

        case SYNC_AP_CMD_EXIT:         // Exit sync mode
            simpliciti_flag |= SIMPLICITI_TRIGGER_STOP;
            break;
    }
}

// *************************************************************************************************
// @fn          simpliciti_sync_get_data_callback
// @brief       For SYNC mode only: Access point has requested data. Copy this data into the TX
// buffer now.
// @param       u16 index               Index used for memory requests
// @return      none
// *************************************************************************************************
void simpliciti_sync_get_data_callback(unsigned int index)
{
    u8 i;

    // simpliciti_data[0] contains data type and needs to be returned to AP
    switch (simpliciti_data[0])
    {
        case SYNC_ED_TYPE_STATUS: // Assemble status packet
            simpliciti_data[1] = (sys.flag.use_metric_units << 7) | (sTime.hour & 0x7F);
            simpliciti_data[2] = sTime.minute;
            simpliciti_data[3] = sTime.second;
            simpliciti_data[4] = sDate.year >> 8;
            simpliciti_data[5] = sDate.year & 0xFF;
            simpliciti_data[6] = sDate.month;
            simpliciti_data[7] = sDate.day;
            simpliciti_data[8] = sAlarm.hour;
            simpliciti_data[9] = sAlarm.minute;
            simpliciti_data[10] = sTemp.degrees >> 8;
            simpliciti_data[11] = sTemp.degrees & 0xFF;
            //      simpliciti_data[12] = sAlt.altitude >> 8;
            //      simpliciti_data[13] = sAlt.altitude & 0xFF;
            break;

        case SYNC_ED_TYPE_MEMORY:
            if (burst_mode == 1)
            {
                // Set burst packet address
                simpliciti_data[1] = ((burst_start + index) >> 8) & 0xFF;
                simpliciti_data[2] = (burst_start + index) & 0xFF;
                // Assemble payload
                for (i = 3; i < BM_SYNC_DATA_LENGTH; i++)
                    simpliciti_data[i] = index;
            }
            else if (burst_mode == 2)
            {
                // Set burst packet address
                simpliciti_data[1] = (burst_packet[index] >> 8) & 0xFF;
                simpliciti_data[2] = burst_packet[index] & 0xFF;
                // Assemble payload
                for (i = 3; i < BM_SYNC_DATA_LENGTH; i++)
                    simpliciti_data[i] = index;
            }
            break;
    }
}

// *************************************************************************************************
// @fn          simpliciti_2chronos_decode_data_callback
// @brief       For 2chronos mode only: Trigger action after receive data from other watch
// @param       none
// @return      none
// *************************************************************************************************
void simpliciti_2chronos_decode_data_callback(void)
{
    sRF2chronos.error_in_messages = 0;
    switch (simpliciti_data[0])
    {
        case SYNC_2CHRONOS_CMD_NOP:
            break;

        case SYNC_2CHRONOS_CMD_DATA_BUTTON: // Get data
            switch (simpliciti_data[1])
            {                               // If the other watch pressed a STAR button
                case SIMPLICITI_BUTTON_STAR:
                    break;

                // If the other watch pressed a NUM button
                case SIMPLICITI_BUTTON_NUM:
                    oslo_conference();
                    break;
            }
            break;

        case SYNC_2CHRONOS_CMD_DATA_SYNC: // Get data
            //rf_send_time = sTime.system_time%SIMPLICITI_BIKE_SEND_INTERVAL;
            // Sync timers from watches

            // Stop Timer0
            TA0CTL &= ~MC_2;

            // Set Timer0. The 0x700 is an experimental offset to make sure the listener timer
            // interruption occurs some miliseconds before the linker
            //TA0R = ((u16)((simpliciti_data[2] << 8) + simpliciti_data[1]) + 0x700);
            TA0R = ((u16)((simpliciti_data[2] << 8) + simpliciti_data[1]) + 0x150);
            TA0CCR0 = (TA0R < 0x7FFF) ? (0x7FFF) : (0xFFFF);

            // Release Timer
            TA0CTL |= MC_2;

            sRF2chronos.RF_2chronos_timer_offset = sTime.system_time % SIMPLICITI_2CHRONOS_INTERVAL;

            simpliciti_data[0] = SYNC_2CHRONOS_CMD_ACK;
            break;

        case SYNC_2CHRONOS_CMD_DISCONNECT:
            simpliciti_flag = SIMPLICITI_TRIGGER_STOP;
            break;

    }
}

// *************************************************************************************************
// @fn          simpliciti_2chronos_get_data_callback
// @brief       For 2chronos mode only: Other watch requested data
// @param       none
// @return      none
// *************************************************************************************************
void simpliciti_2chronos_get_data_callback(void)
{
    u16 actual_timer_value = 0;

    // simpliciti_data[0] contains data type and needs to be returned to AP
    switch (simpliciti_data[0])
    {
        case SYNC_2CHRONOS_CMD_NOP:
            break;

        case SYNC_2CHRONOS_CMD_DATA_BUTTON:                        // Get data

            simpliciti_data[1] = 0;
            if (sRF2chronos.RF_2chronos_button > 0)
            {
                simpliciti_data[1] = sRF2chronos.RF_2chronos_button;
                sRF2chronos.RF_2chronos_button = 0;
            }

            // Number of button presses that occurred
            simpliciti_data[2] = sRF2chronos.c2c_send_value;

            break;

        case SYNC_2CHRONOS_CMD_DATA_SYNC:                          // Get data

            while (actual_timer_value != TA0R)
                actual_timer_value = TA0R;

            simpliciti_data[1] =  actual_timer_value & 0xFF;       // sync timers
            simpliciti_data[2] = (actual_timer_value >> 8) & 0xFF; // sync timers
            sRF2chronos.RF_2chronos_timer_offset = sTime.system_time % SIMPLICITI_2CHRONOS_INTERVAL;

            break;

        case SYNC_2CHRONOS_CMD_DISCONNECT:
            simpliciti_flag = SIMPLICITI_TRIGGER_STOP;
            break;
    }
}

// *************************************************************************************************
// @fn          update_simpliciti_timer
// @brief       Update the register value for the timer interrupt
// @param       none
// @return      none
// *************************************************************************************************
void update_simpliciti_timer(void)
{
    u16 value;

    value = TA0CCR2 + SIMPLICITI_2CHRONOS_TICK;

    // Update CCR
    TA0CCR2 = value;
}

// *************************************************************************************************
// @fn          start_simpliciti_sync_timer
// @brief       Starts simpliciti sync timer interrupt
// @param       none
// @return      none
// *************************************************************************************************
void start_simpliciti_sync_timer(void)
{
    // Init CCR register with current time
    TA0CCR2 = TA0R;

    update_simpliciti_timer();

    // Reset IRQ flag
    TA0CCTL2 &= ~CCIFG;

    // Enable timer interrupt
    TA0CCTL2 |= CCIE;
}

// *************************************************************************************************
// @fn          stop_simpliciti_sync_timer
// @brief       Stops simpliciti sync timer interrupt
// @param       none
// @return      none
// *************************************************************************************************
void stop_simpliciti_sync_timer(void)
{
    // Clear timer interrupt enable
    TA0CCTL2 &= ~CCIE;
}

// *************************************************************************************************
// @fn          start_simpliciti_2chronos
// @brief       Function responsable to handle Chronos-Chronos communication
// @param       none
// @return      none
// *************************************************************************************************
void start_simpliciti_2chronos(void)
{
    // Display time in line 1
    clear_line(LINE1);
    fptr_lcd_function_line1(LINE1, DISPLAY_LINE_CLEAR);

    // Turn on beeper icon to show activity
    display_symbol(LCD_ICON_BEEPER1, SEG_ON_BLINK_ON);
    display_symbol(LCD_ICON_BEEPER2, SEG_ON_BLINK_ON);
    display_symbol(LCD_ICON_BEEPER3, SEG_ON_BLINK_ON);

    display_chars(LCD_SEG_L1_3_0, (u8*) "FIND", SEG_ON);

    // Debounce button event
    Timer0_A4_Delay(CONV_MS_TO_TICKS(BUTTONS_DEBOUNCE_TIME_OUT));

    // Prepare radio for RF communication
    open_radio();

    // Start sensor
    as_start();

    // Enable timer interrupt to change listen/link modes
    start_simpliciti_sync_timer();

    // Set SimpliciTI mode
    sRFsmpl.mode = SIMPLICITI_BUTTONS;

    // Set timeout counter
    sAccel.timeout = ACCEL_MEASUREMENT_TIMEOUT;

    // Set mode
    sAccel.mode = ACCEL_MODE_ON;

    // create a device address for simpliciti communication
    set_simpliciti_ed_address();

    // Set SimpliciTI timeout to save battery power - 60 minutes
    sRFsmpl.timeout = SIMPLICITI_TIMEOUT;

    // Set Chronos-chronos connection timeout to save battery power - 15 seconds
    sRF2chronos.timeout = SIMPLICITI_2CHRONOS_TIMEOUT;

    // Value to be displayed in the LCD
    sRF2chronos.c2c_value = 0;

    // Button that was pressed
    sRF2chronos.RF_2chronos_button = 0;

    // Variable used to synchronize the timers from both watches
    sRF2chronos.RF_2chronos_timer_offset = 0;

    // Number of problems detected - miss message
    sRF2chronos.error_in_messages = 0;

    if (simpliciti_2chronos())
    {
        // Stop timer interrupt used to change between Listen and Link modes
        stop_simpliciti_sync_timer();

        display_chars(LCD_SEG_L1_3_0, (u8*) "C2C", SEG_ON);
        display_chars(LCD_SEG_L2_5_0, (u8 *) " FOUND", SEG_ON);
        simpliciti_data[0] = SYNC_2CHRONOS_CMD_DATA_BUTTON;

        while (1)
        {
            // If it is time to send the data
            if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_DATA))
            {
                if (sRF2chronos.error_in_messages <= SIMPLICITI_2CHRONOS_MISS_COUNT)
                {
                    // Turn on the beeper lcd icon to show user that the watch is sending data
                    display_symbol(LCD_ICON_BEEPER1, SEG_ON);
                    display_symbol(LCD_ICON_BEEPER2, SEG_ON);
                    display_symbol(LCD_ICON_BEEPER3, SEG_ON);

                    // Check what kind of device we are dealing with and trigger send/receive action
                    if (getFlag(simpliciti_flag, SIMPLICITI_2CHRONOS_LISTEN))
                    {
                        // Send data to other watch and receive answer
                        simpliciti_control_other_watch();
                    }
                    else if (getFlag(simpliciti_flag, SIMPLICITI_2CHRONOS_LINK))
                    {
                        // Receive data from other watch and send answer
                        simpliciti_receive_command_from_other_watch();
                    }

                    // Turn off beeper icon after data was sent
                    display_symbol(LCD_ICON_BEEPER1, SEG_OFF);
                    display_symbol(LCD_ICON_BEEPER2, SEG_OFF);
                    display_symbol(LCD_ICON_BEEPER3, SEG_OFF);

                    // Clear flag before get in LPM mode
                    clearFlag(simpliciti_flag, SIMPLICITI_TRIGGER_SEND_DATA);
                }
                else
                {
                    break;
                }
            }
            // Device stays in LPM mode and it is awaken by timer and port interrupts
            _BIS_SR(LPM3_bits + GIE);
            __no_operation();

            // Exit method
            if (getFlag(simpliciti_flag, SIMPLICITI_TRIGGER_STOP))
            {
                break;
            }
            WDTCTL = WDTPW + WDTIS__512K + WDTSSEL__ACLK + WDTCNTCL;
        }
    }

    reset_simpliciti();

    // Set SimpliciTI state to OFF
    sRFsmpl.mode = SIMPLICITI_OFF;

    // Stop acceleration sensor
    as_stop();

    // Powerdown radio
    close_radio();

    // Clear last button events
    Timer0_A4_Delay(CONV_MS_TO_TICKS(BUTTONS_DEBOUNCE_TIME_OUT));
    BUTTONS_IFG = 0x00;
    button.all_flags = 0;

    // Clear icons
    display_symbol(LCD_ICON_BEEPER1, SEG_OFF_BLINK_OFF);
    display_symbol(LCD_ICON_BEEPER2, SEG_OFF_BLINK_OFF);
    display_symbol(LCD_ICON_BEEPER3, SEG_OFF_BLINK_OFF);

    // Clean up line 1
    clear_line(LINE1);
    display_time(LINE1, DISPLAY_LINE_CLEAR);

    // Force full display update
    display.flag.full_update = 1;
}

// *************************************************************************************************
// @fn          change_link_listen
// @brief       For 2chronos mode only: During the connection, each 100ms the timer interrupt calls
// this function.
//              In the C2C connection, one device must be listening while the other tries to link.
//              This function change the type of connection based on the accelerometer data.
// @param       none
// @return      none
// *************************************************************************************************
void change_link_listen(void)
{
    // Get data from sensor
    as_get_data(sAccel.xyz);

    // If the data is an even number
    if ((sAccel.xyz[0] % 2) == 0)
    {
        // Set listen and clear linker flag
        simpliciti_flag &= ~SIMPLICITI_2CHRONOS_LINK;
        simpliciti_flag |= SIMPLICITI_2CHRONOS_LISTEN;
    }
    // If the number is an odd number
    else
    {
        // Set linker and clear listen flag
        simpliciti_flag &= ~SIMPLICITI_2CHRONOS_LISTEN;
        simpliciti_flag |= SIMPLICITI_2CHRONOS_LINK;
    }
}

// *************************************************************************************************
// @fn          set_simpliciti_ed_address
// @brief       Each device address must be unique. This functions creates an address based on the
// current time
// @param       none
// @return      none
// *************************************************************************************************
u8 set_simpliciti_ed_address(void)
{
    Timer0_A4_Delay(CONV_MS_TO_TICKS(250));
    as_get_data(sAccel.xyz);

    simpliciti_ed_address[0] = (sTime.system_time % 7) * 10 + sTime.system_time % 3;
    simpliciti_ed_address[1] = sAccel.xyz[1];
    simpliciti_ed_address[2] = sAccel.xyz[2];
    simpliciti_ed_address[3] = sTime.second;
    return 1;
}

// *************************************************************************************************
// @fn          oslo_conference
// @brief       Special function designed for the conference in oslo
// @param       none
// @return      none
// *************************************************************************************************
void oslo_conference(void)
{
    // auxiliar variable
    u8 *string;
    u8 chronosChannel[3];

    // counter
    sRF2chronos.c2c_value++;

    display_chars(LCD_SEG_L1_3_0, (u8 *) "C2C", SEG_ON);

    // display counter
    string = int_to_array(sRF2chronos.c2c_value, 2, 0);

    display_chars(LCD_SEG_L2_1_0, string, SEG_ON);

    // Display channel
    if ((SMARTRF_SETTING_CHANNR >> 4) < 0x0A)
        chronosChannel[0] = '0' + (SMARTRF_SETTING_CHANNR >> 4);           //LSB is less than 10
    else
        chronosChannel[0] = 'A' - 0x0A + (SMARTRF_SETTING_CHANNR >> 4);    //LSB is greater than 10

    if ((SMARTRF_SETTING_CHANNR & 0x0F) < 0x0A)
        chronosChannel[1] = '0' + (SMARTRF_SETTING_CHANNR & 0x0F);         //LSB is less than 10
    else
        chronosChannel[1] = 'A' - 0x0A + (SMARTRF_SETTING_CHANNR & 0x0F);  //LSB is greater than 10
    chronosChannel[2] = '-';
    display_chars(LCD_SEG_L2_4_2, (u8 *) chronosChannel, SEG_ON);
}
