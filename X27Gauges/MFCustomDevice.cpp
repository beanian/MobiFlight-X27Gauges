#include "MFCustomDevice.h"
#include "commandmessenger.h"
#include "allocateMem.h"
#include "MFEEPROM.h"
#if defined(HAS_CONFIG_IN_FLASH)
#include "MFCustomDevicesConfig.h"
#else
const char CustomDeviceConfig[] PROGMEM = {};
#endif

extern MFEEPROM MFeeprom;

/* **********************************************************************************
    The custom device pins, type and configuration is stored in the EEPROM
    While loading the config the adresses in the EEPROM are transferred to the constructor
    Within the constructor you have to copy the EEPROM content to a buffer
    and evaluate him. The buffer is used for all 3 types (pins, type configuration),
    so do it step by step.
    The max size of the buffer is defined here. It must be the size of the
    expected max length of these strings.

    3 pins with two digits each, delimited by "|" plus NULL -> 9 bytes
    The custom type is "X27_POSTEP_VID6606" -> 19 bytes
    The configuration is "<speed>|<accel>", e.g. "280|3000" -> 10 bytes
********************************************************************************** */
#define MEMLEN_STRING_BUFFER 40

// reads a string from EEPROM or Flash at given address which is '.' terminated and saves it to the buffer
bool MFCustomDevice::getStringFromMem(uint16_t addrMem, char *buffer, bool configFromFlash)
{
    char     temp     = 0;
    uint8_t  counter  = 0;
    uint16_t length   = MFeeprom.get_length();
    do {
        if (configFromFlash) {
            temp = pgm_read_byte_near(CustomDeviceConfig + addrMem++);
            if (addrMem > sizeof(CustomDeviceConfig))
                return false;
        } else {
            temp = MFeeprom.read_byte(addrMem++);
            if (addrMem > length)
                return false;
        }
        buffer[counter++] = temp;              // save character and locate next buffer position
        if (counter >= MEMLEN_STRING_BUFFER) { // nameBuffer will be exceeded
            return false;                      // abort copying to buffer
        }
    } while (temp != '.'); // reads until limiter '.' and locates the next free buffer position
    buffer[counter - 1] = 0x00; // replace '.' by NULL, terminates the string
    return true;
}

MFCustomDevice::MFCustomDevice()
{
    _initialized = false;
}

/* **********************************************************************************
    Within the connector pins, a device name and a config string can be defined
    These informations are stored in the EEPROM or Flash like for the other devices.
    While reading the config from the EEPROM or Flash this function is called.
    It is the first function which will be called for the custom device.
    If it fits into the memory buffer, the constructor for the customer device
    will be called
********************************************************************************** */

void MFCustomDevice::attach(uint16_t adrPin, uint16_t adrType, uint16_t adrConfig, bool configFromFlash)
{
    if (adrPin == 0) return;

    char  *params, *p = NULL;
    char   parameter[MEMLEN_STRING_BUFFER];
    uint8_t pins[3] = {0, 0, 0};

    if (!getStringFromMem(adrType, parameter, configFromFlash) || strcmp(parameter, "X27_POSTEP_VID6606") != 0) {
        cmdMessenger.sendCmd(kStatus, F("Custom Device is not supported by this firmware version"));
        return;
    }

    void *mem = MF_ALLOC_TYPE(PoStepVID6606, 1);
    if (!mem) {
        cmdMessenger.sendCmd(kStatus, F("Custom Device does not fit in Memory"));
        return;
    }

    // Pins in the order of the device.json: SER | SCK | RCK
    if (!getStringFromMem(adrPin, parameter, configFromFlash))
        return;
    params = strtok_r(parameter, "|", &p);
    for (uint8_t i = 0; i < 3 && params; i++) {
        pins[i] = atoi(params);
        params  = strtok_r(NULL, "|", &p);
    }

    // Optional config string "<max speed deg/s>|<acceleration deg/s^2>",
    // anything missing falls back to the build defaults
    uint16_t speed = X27_DEFAULT_SPEED;
    uint16_t accel = X27_DEFAULT_ACCEL;
    if (getStringFromMem(adrConfig, parameter, configFromFlash)) {
        params = strtok_r(parameter, "|", &p);
        if (params && atoi(params) > 0) speed = atoi(params);
        params = strtok_r(NULL, "|", &p);
        if (params && isdigit(*params)) accel = atol(params);
    }

    _board = new (mem) PoStepVID6606(pins[0], pins[1], pins[2]);
    _board->attach(speed, accel);
    _initialized = true;
}

/* **********************************************************************************
    The custom devives gets unregistered if a new config gets uploaded.
    It gets called from CustomerDevice::Clear()
********************************************************************************** */
void MFCustomDevice::detach()
{
    if (!_initialized) return;
    _initialized = false;
    _board->detach();
}

/* **********************************************************************************
    Gets called every loop(). Stepping normally runs from a timer interrupt,
    this only does work in the X27_NO_TIMER build.
********************************************************************************** */
void MFCustomDevice::update()
{
    if (!_initialized) return;
    _board->update();
}

/* **********************************************************************************
    If an output for the custom device is defined in the connector,
    this function gets called when a new value is available.
    It gets called from CustomerDevice::OnSet()
********************************************************************************** */
void MFCustomDevice::set(int16_t messageID, char *setPoint)
{
    if (!_initialized) return;
    _board->set(messageID, setPoint);
}
