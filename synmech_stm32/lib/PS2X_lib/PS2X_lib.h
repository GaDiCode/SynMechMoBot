/******************************************************************
*  Super amazing PS2 controller Arduino Library v1.8
*   details and example sketch:
*       http://www.billporter.info/?p=240
*
*   Original code by Shutter on Arduino Forums
*   Revamped, made into lib by Bill Porter (www.billporter.info)
*   Contributors: Eric Wetzel, Kurt Eckhardt
*
*   Modified for STM32 compatibility:
*     - Replaced direct AVR/PIC32 port manipulation with digitalWrite()
*     - Replaced deprecated 'boolean' with 'bool'
*     - Added robot control helper methods
*
*   License: GPL v3 <http://www.gnu.org/licenses/>
******************************************************************/

//#define PS2X_DEBUG
//#define PS2X_COM_DEBUG

#ifndef PS2X_lib_h
#define PS2X_lib_h

#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdint.h>

#define CTRL_CLK        4
#define CTRL_CLK_HIGH   4
#define CTRL_BYTE_DELAY 4

// Button constants
#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_GREEN       0x1000
#define PSB_RED         0x2000
#define PSB_BLUE        0x4000
#define PSB_PINK        0x8000
#define PSB_TRIANGLE    0x1000
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000
#define PSB_SQUARE      0x8000

// Guitar button constants
#define GREEN_FRET      0x0200
#define RED_FRET        0x2000
#define YELLOW_FRET     0x1000
#define BLUE_FRET       0x4000
#define ORANGE_FRET     0x8000
#define STAR_POWER      0x0100
#define UP_STRUM        0x0010
#define DOWN_STRUM      0x0040
#define WHAMMY_BAR      8

// Stick values
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

// Analog buttons
#define PSAB_PAD_RIGHT   9
#define PSAB_PAD_UP      11
#define PSAB_PAD_DOWN    12
#define PSAB_PAD_LEFT    10
#define PSAB_L2          19
#define PSAB_R2          20
#define PSAB_L1          17
#define PSAB_R1          18
#define PSAB_GREEN       13
#define PSAB_RED         14
#define PSAB_BLUE        15
#define PSAB_PINK        16
#define PSAB_TRIANGLE    13
#define PSAB_CIRCLE      14
#define PSAB_CROSS       15
#define PSAB_SQUARE      16

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x & (1<<y))
#define TOG(x,y) (x^=(1<<y))

class PS2X {
public:
    bool Button(uint16_t);
    unsigned int ButtonDataByte();
    bool NewButtonState();
    bool NewButtonState(unsigned int);
    bool ButtonPressed(unsigned int);
    bool ButtonReleased(unsigned int);
    void read_gamepad();
    bool read_gamepad(bool, byte);
    byte readType();
    byte config_gamepad(uint8_t, uint8_t, uint8_t, uint8_t);
    byte config_gamepad(uint8_t, uint8_t, uint8_t, uint8_t, bool, bool);
    void enableRumble();
    bool enablePressures();
    byte Analog(byte);
    void reconfig_gamepad();

    // Robot control helpers
    float getRobotLinearVel(float maxVel, uint8_t deadZone);
    float getRobotAngularVel(float maxVel, uint8_t deadZone);
    bool  isEmergencyStop();
    float getSpeedMultiplier();

private:
    float mapJoystickQuadratic(uint8_t raw, float maxVal, uint8_t deadZone);

    // Simple pin-based I/O (platform-independent)
    inline void CLK_SET(void) { digitalWrite(_clk_pin, HIGH); }
    inline void CLK_CLR(void) { digitalWrite(_clk_pin, LOW);  }
    inline void CMD_SET(void) { digitalWrite(_cmd_pin, HIGH); }
    inline void CMD_CLR(void) { digitalWrite(_cmd_pin, LOW);  }
    inline void ATT_SET(void) { digitalWrite(_att_pin, HIGH); }
    inline void ATT_CLR(void) { digitalWrite(_att_pin, LOW);  }
    inline bool DAT_CHK(void) { return digitalRead(_dat_pin); }

    unsigned char _gamepad_shiftinout(char);
    unsigned char PS2data[21];
    void sendCommandString(byte*, byte);
    unsigned char i;
    unsigned int last_buttons;
    unsigned int buttons;

    // Store pin numbers instead of port registers
    uint8_t _clk_pin;
    uint8_t _cmd_pin;
    uint8_t _att_pin;
    uint8_t _dat_pin;

    unsigned long last_read;
    byte read_delay;
    byte controller_type;
    bool en_Rumble;
    bool en_Pressures;
};

#endif
