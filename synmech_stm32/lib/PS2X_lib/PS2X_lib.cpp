#include "PS2X_lib.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

static byte enter_config[]={0x01,0x43,0x00,0x01,0x00};
static byte set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
static byte set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
static byte exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
static byte enable_rumble[]={0x01,0x4D,0x00,0x00,0x01};
static byte type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};

bool PS2X::NewButtonState() {
    return ((last_buttons ^ buttons) > 0);
}

bool PS2X::NewButtonState(unsigned int button) {
    return (((last_buttons ^ buttons) & button) > 0);
}

bool PS2X::ButtonPressed(unsigned int button) {
    return(NewButtonState(button) & Button(button));
}

bool PS2X::ButtonReleased(unsigned int button) {
    return((NewButtonState(button)) & ((~last_buttons & button) > 0));
}

bool PS2X::Button(uint16_t button) {
    return ((~buttons & button) > 0);
}

unsigned int PS2X::ButtonDataByte() {
    return (~buttons);
}

byte PS2X::Analog(byte button) {
    return PS2data[button];
}

unsigned char PS2X::_gamepad_shiftinout(char byte) {
    unsigned char tmp = 0;
    for(i=0;i<8;i++) {
        if(CHK(byte,i)) CMD_SET();
        else CMD_CLR();
        CLK_CLR();
        delayMicroseconds(CTRL_CLK);
        if(DAT_CHK()) SET(tmp,i);
        CLK_SET();
        delayMicroseconds(CTRL_CLK_HIGH);
    }
    CMD_SET();
    delayMicroseconds(CTRL_BYTE_DELAY);
    return tmp;
}

void PS2X::read_gamepad() {
    read_gamepad(false, 0x00);
}

bool PS2X::read_gamepad(bool motor1, byte motor2) {
    double temp = millis() - last_read;

    if (temp > 1500)
        reconfig_gamepad();

    if(temp < read_delay)
        delay(read_delay - temp);

    if(motor2 != 0x00)
        motor2 = map(motor2,0,255,0x40,0xFF);

    byte dword[9] = {0x01,0x42,0,motor1,motor2,0,0,0,0};
    byte dword2[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

    for (byte RetryCnt = 0; RetryCnt < 5; RetryCnt++) {
        CMD_SET();
        CLK_SET();
        ATT_CLR();
        delayMicroseconds(CTRL_BYTE_DELAY);

        for (int i = 0; i<9; i++) {
            PS2data[i] = _gamepad_shiftinout(dword[i]);
        }

        if(PS2data[1] == 0x79) {
            for (int i = 0; i<12; i++) {
                PS2data[i+9] = _gamepad_shiftinout(dword2[i]);
            }
        }

        ATT_SET();
        if ((PS2data[1] & 0xf0) == 0x70)
            break;

        reconfig_gamepad();
        delay(read_delay);
    }

    if ((PS2data[1] & 0xf0) != 0x70) {
        if (read_delay < 10)
            read_delay++;
    }

#ifdef PS2X_COM_DEBUG
    Serial.println("OUT:IN");
    for(int i=0; i<9; i++){
        Serial.print(dword[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i], HEX);
        Serial.print(" ");
    }
    for (int i = 0; i<12; i++) {
        Serial.print(dword2[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i+9], HEX);
        Serial.print(" ");
    }
    Serial.println("");
#endif

    last_buttons = buttons;
    buttons = (uint16_t)(PS2data[4] << 8) + PS2data[3];
    last_read = millis();
    return ((PS2data[1] & 0xf0) == 0x70);
}

byte PS2X::config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat) {
    return config_gamepad(clk, cmd, att, dat, false, false);
}

byte PS2X::config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble) {
    byte temp[sizeof(type_read)];

    // Store pin numbers for digitalWrite-based I/O
    _clk_pin = clk;
    _cmd_pin = cmd;
    _att_pin = att;
    _dat_pin = dat;

    pinMode(clk, OUTPUT);
    pinMode(att, OUTPUT);
    pinMode(cmd, OUTPUT);
    pinMode(dat, INPUT_PULLUP);

    CMD_SET();
    CLK_SET();

    read_gamepad();
    read_gamepad();

    if(PS2data[1] != 0x41 && PS2data[1] != 0x73 && PS2data[1] != 0x79){
#ifdef PS2X_DEBUG
        Serial.println("Controller mode not matched or no controller found");
        Serial.print("Expected 0x41 or 0x73, got ");
        Serial.println(PS2data[1], HEX);
#endif
        return 1;
    }

    read_delay = 1;

    for(int y = 0; y <= 10; y++)
    {
        sendCommandString(enter_config, sizeof(enter_config));
        delayMicroseconds(CTRL_BYTE_DELAY);

        CMD_SET();
        CLK_SET();
        ATT_CLR();
        delayMicroseconds(CTRL_BYTE_DELAY);

        for (int i = 0; i<9; i++) {
            temp[i] = _gamepad_shiftinout(type_read[i]);
        }

        ATT_SET();
        controller_type = temp[3];

        sendCommandString(set_mode, sizeof(set_mode));
        if(rumble){ sendCommandString(enable_rumble, sizeof(enable_rumble)); en_Rumble = true; }
        if(pressures){ sendCommandString(set_bytes_large, sizeof(set_bytes_large)); en_Pressures = true; }
        sendCommandString(exit_config, sizeof(exit_config));

        read_gamepad();

        if(pressures){
            if(PS2data[1] == 0x79)
                break;
            if(PS2data[1] == 0x73)
                return 3;
        }

        if(PS2data[1] == 0x73)
            break;

        if(y == 10){
#ifdef PS2X_DEBUG
            Serial.println("Controller not accepting commands");
            Serial.print("mode still set at ");
            Serial.println(PS2data[1], HEX);
#endif
            return 2;
        }

        read_delay += 1;
    }

    return 0;
}

void PS2X::sendCommandString(byte string[], byte len) {
#ifdef PS2X_COM_DEBUG
    byte temp[len];
    ATT_CLR();
    delayMicroseconds(CTRL_BYTE_DELAY);
    for (int y=0; y < len; y++)
        temp[y] = _gamepad_shiftinout(string[y]);
    ATT_SET();
    delay(read_delay);

    Serial.println("OUT:IN Configure");
    for(int i=0; i<len; i++){
        Serial.print(string[i], HEX);
        Serial.print(":");
        Serial.print(temp[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
#else
    ATT_CLR();
    for (int y=0; y < len; y++)
        _gamepad_shiftinout(string[y]);
    ATT_SET();
    delay(read_delay);
#endif
}

byte PS2X::readType() {
    if(controller_type == 0x03)
        return 1;
    else if(controller_type == 0x01)
        return 2;
    return 0;
}

void PS2X::enableRumble() {
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(enable_rumble, sizeof(enable_rumble));
    sendCommandString(exit_config, sizeof(exit_config));
    en_Rumble = true;
}

bool PS2X::enablePressures() {
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));

    read_gamepad();
    read_gamepad();

    if(PS2data[1] != 0x79)
        return false;

    en_Pressures = true;
    return true;
}

void PS2X::reconfig_gamepad(){
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_mode, sizeof(set_mode));
    if (en_Rumble)
        sendCommandString(enable_rumble, sizeof(enable_rumble));
    if (en_Pressures)
        sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));
}

// ==================== Robot Control Helpers ====================

float PS2X::mapJoystickQuadratic(uint8_t rawValue, float maxVel, uint8_t deadZone) {
    int offset = (int)rawValue - 128;
    if (abs(offset) < deadZone) {
        return 0.0f;
    }
    float normalized = 0.0f;
    if (offset > 0) {
        normalized = (float)(offset - deadZone) / (127.0f - deadZone);
    } else {
        normalized = (float)(offset + deadZone) / (128.0f - deadZone);
    }
    float sign = (normalized > 0) ? 1.0f : -1.0f;
    return sign * (normalized * normalized) * maxVel;
}

float PS2X::getRobotLinearVel(float maxVel, uint8_t deadZone) {
    uint8_t rawY = Analog(PSS_LY);
    float vel = mapJoystickQuadratic(rawY, maxVel, deadZone);
    return -vel;  // Invert: stick up = forward
}

float PS2X::getRobotAngularVel(float maxVel, uint8_t deadZone) {
    uint8_t rawX = Analog(PSS_LX);
    float vel = mapJoystickQuadratic(rawX, maxVel, deadZone);
    return -vel;  // Invert: stick left = turn left (positive angular)
}

bool PS2X::isEmergencyStop() {
    return Button(PSB_SELECT);
}

float PS2X::getSpeedMultiplier() {
    if (Button(PSB_L1)) return 0.5f;
    if (Button(PSB_R1)) return 2.0f;
    return 1.0f;
}
