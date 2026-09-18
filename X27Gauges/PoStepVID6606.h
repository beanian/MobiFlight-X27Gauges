#pragma once

#include "Arduino.h"

/* **********************************************************************************
    PoLabs PoStepVID6606 driver board: two VID6606 chips behind a 16 bit shift
    register (SER / SCK / RCK), driving up to 8 X27.168 gauge steppers.

    One instance = one board. Motor numbers are the labels printed on the board
    (Motor1 .. Motor8), NOT the screw terminal positions.

    All instances share one hardware timer which generates the step pulses, so the
    needles keep moving smoothly while the MobiFlight main loop is busy.
********************************************************************************** */

// ---------- build time options (set via -D in X27Gauges_platformio.ini) ----------
#ifndef X27_TIMER
#define X27_TIMER 4 // 16 bit timer used for stepping: 1, 3, 4 or 5 (Servos use 5)
#endif
#ifndef X27_DEFAULT_SPEED
#define X27_DEFAULT_SPEED 280 // deg/s
#endif
#ifndef X27_DEFAULT_ACCEL
#define X27_DEFAULT_ACCEL 3000 // deg/s^2, 0 = no ramp
#endif
#ifndef X27_START_SPEED
#define X27_START_SPEED 60 // deg/s, speed a needle may start/stop/reverse at without a ramp
#endif
#ifndef X27_HOME_SPEED
#define X27_HOME_SPEED 90 // deg/s while driving into the zero stop
#endif
#ifndef X27_DIR_TOWARD_ZERO
#define X27_DIR_TOWARD_ZERO 0 // DIR bit value which moves the needle toward the zero stop
#endif

// ---------- motor geometry ----------
// VID6606 moves the X27 1/12 degree per pulse
#define X27_STEPS_PER_DEG   12
#define X27_MAX_POS         (315 * X27_STEPS_PER_DEG) // 3780 = full 315 degree sweep
#define X27_HOME_OVERTRAVEL (X27_MAX_POS + 400)       // guarantees the stop is reached
#define X27_MOTORS          8
#define X27_MAX_BOARDS      4
#define X27_RAMP_TABLE      128

// ---------- message IDs, must match x27_postep_vid6606.device.json ----------
enum {
    X27_MSG_STOP      = -1, // Connector stopped
    X27_MSG_POWERSAVE = -2, // "1" entering, "0" leaving
    X27_MSG_DEG_FIRST = 1,  // 1..8   -> Motor 1..8 angle in degrees (decimals allowed)
    X27_MSG_RAW_FIRST = 11, // 11..18 -> Motor 1..8 position in microsteps
    X27_MSG_HOME      = 20, // 0 = all motors, 1..8 = single motor
    X27_MSG_SPEED     = 21, // max speed deg/s
    X27_MSG_ACCEL     = 22  // acceleration deg/s^2, 0 = no ramp
};

class PoStepVID6606
{
public:
    PoStepVID6606(uint8_t pinSer, uint8_t pinSck, uint8_t pinRck);
    void attach(uint16_t speedDeg, uint16_t accelDeg);
    void detach();
    void set(int16_t messageID, char *setPoint);
    void update();

    static void tickAll(); // one step period, called from the timer ISR

private:
    struct Motor {
        int16_t  pos;    // pulses sent so far, there is no position feedback
        int16_t  target; // commanded position
        uint16_t phase;  // DDS accumulator, overflow = step
        uint16_t ramp;   // steps needed to brake to start speed
        int8_t   dir;    // -1, 0 (stopped), +1
        bool     homing;
    };

    void tick();
    void sendWord(uint16_t w);
    void setTarget(uint8_t motor, int16_t steps);
    void home(uint8_t motor);
    void homeAll();

    static void    configureMotion(uint16_t speedDeg, uint16_t accelDeg);
    static int16_t degToSteps(const char *s);

    volatile uint8_t *_serPort, *_sckPort, *_rckPort;
    uint8_t           _serMask, _sckMask, _rckMask;
    uint8_t           _pinSer, _pinSck, _pinRck;
    uint16_t          _word; // last word latched into the board
    Motor             _motors[X27_MOTORS];
    volatile bool     _active; // false while every needle is at its target
    volatile bool     _parked; // power saving: hold all needles at zero
    bool              _initialised;
};
