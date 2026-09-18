#include "PoStepVID6606.h"
#include <util/atomic.h>
#include <math.h>

/* **********************************************************************************
    Step timing

    A 16 bit timer runs in CTC mode with ICRn as TOP and fires once per step period
    at max needle speed (280 deg/s -> 297 us). Using ICRn instead of OCRnA keeps the
    period safe from a stray analogWrite() on one of the timer's PWM pins.

    Each motor has a DDS accumulator: every tick the current speed (as a fraction of
    max speed, 0xFFFF = one step per tick) is added, an overflow means "step now".
    At max speed that gives perfectly even pulses, below it the pulses snap to the
    tick grid, which the needle's inertia hides.

    Acceleration uses the classic "steps since start" ramp: _ramp counts the steps
    needed to brake back to start speed and indexes a table of speeds, so there is
    no multiplication or division inside the ISR.
********************************************************************************** */

// ---------- register names for the selected timer ----------
#define X27_CAT3_(a, b, c) a##b##c
#define X27_CAT3(a, b, c)  X27_CAT3_(a, b, c)
#define X27_TCCRA          X27_CAT3(TCCR, X27_TIMER, A)
#define X27_TCCRB          X27_CAT3(TCCR, X27_TIMER, B)
#define X27_TCNT           X27_CAT3(TCNT, X27_TIMER, )
#define X27_ICR            X27_CAT3(ICR, X27_TIMER, )
#define X27_TIMSK          X27_CAT3(TIMSK, X27_TIMER, )
#define X27_ICIE           X27_CAT3(ICIE, X27_TIMER, )
#define X27_WGM0           X27_CAT3(WGM, X27_TIMER, 0)
#define X27_WGM2           X27_CAT3(WGM, X27_TIMER, 2)
#define X27_WGM3           X27_CAT3(WGM, X27_TIMER, 3)
#define X27_CS0            X27_CAT3(CS, X27_TIMER, 0)
#define X27_CS1            X27_CAT3(CS, X27_TIMER, 1)
#define X27_TIMER_VECT     X27_CAT3(TIMER, X27_TIMER, _CAPT_vect)

#define X27_MIN_SPEED 30  // deg/s
#define X27_MAX_SPEED 400 // deg/s -> 208 us tick, leaves the main loop enough time with 16 needles moving
#define X27_MAX_ACCEL 30000

// ---------- state shared by all boards ----------
static PoStepVID6606 *s_boards[X27_MAX_BOARDS];
static uint8_t        s_numBoards = 0;
static uint16_t       s_speedDeg  = X27_DEFAULT_SPEED;
static uint16_t       s_accelDeg  = X27_DEFAULT_ACCEL;
static uint16_t       s_rampInc[X27_RAMP_TABLE]; // speed per ramp index, 0xFFFF = max speed
static uint16_t       s_rampLen;                 // steps from start speed to max speed
static uint8_t        s_rampShift;               // ramp steps per table entry, as a shift
static uint16_t       s_homeInc;
#ifdef X27_NO_TIMER
static uint32_t s_tickUs;
static uint32_t s_lastTickUs;
#endif

// ---------- timer ----------
#ifdef X27_NO_TIMER
static inline void timerMask() {}
static inline void timerUnmask() {}
static inline void timerStart(uint32_t tickUs) { s_tickUs = tickUs; }
static inline void timerStop() {}
#else
static inline void timerMask()
{
    X27_TIMSK &= ~_BV(X27_ICIE);
}

static inline void timerUnmask()
{
    X27_TIMSK |= _BV(X27_ICIE);
}

static void timerStart(uint32_t tickUs)
{
    X27_TCCRA = 0; // releases the timer's PWM pins
    X27_TCCRB = 0;
    X27_TCNT  = 0;
    X27_ICR   = (uint16_t)(tickUs * 2 - 1);                  // prescaler 8 -> 0.5 us per count
    X27_TCCRB = _BV(X27_WGM3) | _BV(X27_WGM2) | _BV(X27_CS1); // CTC, TOP = ICRn
}

static void timerStop()
{
    // back to the Arduino core's default, so PWM on the timer's pins works again
    X27_TCCRB = _BV(X27_CS1) | _BV(X27_CS0);
    X27_TCCRA = _BV(X27_WGM0);
}

ISR(X27_TIMER_VECT)
{
    // Shifting out 16 bits per board takes a while, so let other interrupts
    // (serial RX!) through, but don't re-enter ourselves.
    X27_TIMSK &= ~_BV(X27_ICIE);
    sei();
    PoStepVID6606::tickAll();
    cli();
    X27_TIMSK |= _BV(X27_ICIE);
}
#endif

// ---------- setup ----------
PoStepVID6606::PoStepVID6606(uint8_t pinSer, uint8_t pinSck, uint8_t pinRck)
{
    _pinSer      = pinSer;
    _pinSck      = pinSck;
    _pinRck      = pinRck;
    _initialised = false;
}

static void setupPin(uint8_t pin, volatile uint8_t **port, uint8_t *mask)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    *port = portOutputRegister(digitalPinToPort(pin));
    *mask = digitalPinToBitMask(pin);
}

void PoStepVID6606::attach(uint16_t speedDeg, uint16_t accelDeg)
{
    if (s_numBoards >= X27_MAX_BOARDS)
        return;

    setupPin(_pinSer, &_serPort, &_serMask);
    setupPin(_pinSck, &_sckPort, &_sckMask);
    setupPin(_pinRck, &_rckPort, &_rckMask);

    _word   = 0;
    _parked = false;
    _active = false;
    memset(_motors, 0, sizeof(_motors));
    sendWord(0);

    timerMask();
    s_boards[s_numBoards++] = this;
    configureMotion(speedDeg, accelDeg); // also (re)starts the timer
    _initialised = true;

    // The needle position is unknown after power up. Homing runs from the
    // timer, so the Connector handshake is not blocked for the ~4 s it takes.
    homeAll();
}

void PoStepVID6606::detach()
{
    if (!_initialised)
        return;
    _initialised = false;

    timerMask();
    for (uint8_t i = 0; i < s_numBoards; i++) {
        if (s_boards[i] != this) continue;
        s_boards[i] = s_boards[--s_numBoards];
        break;
    }
    if (s_numBoards == 0)
        timerStop();
    else
        timerUnmask();
}

// Speed and acceleration are shared by all boards as they share the timer.
void PoStepVID6606::configureMotion(uint16_t speedDeg, uint16_t accelDeg)
{
    s_speedDeg = constrain(speedDeg, X27_MIN_SPEED, X27_MAX_SPEED);
    s_accelDeg = min(accelDeg, (uint16_t)X27_MAX_ACCEL);

    float vMax = (float)s_speedDeg * X27_STEPS_PER_DEG; // steps/s, exactly one step per tick
    float v0   = min(vMax, (float)X27_START_SPEED * X27_STEPS_PER_DEG);
    float a    = (float)s_accelDeg * X27_STEPS_PER_DEG; // steps/s^2

    timerMask();

    s_homeInc   = (uint16_t)min(65535.0f, (float)X27_HOME_SPEED / s_speedDeg * 65535.0f);
    s_rampShift = 0;
    if (s_accelDeg == 0 || v0 >= vMax) {
        s_rampLen    = 0;
        s_rampInc[0] = 0xFFFF;
    } else {
        s_rampLen = (uint16_t)min(60000.0f, (vMax * vMax - v0 * v0) / (2 * a));
        while ((s_rampLen >> s_rampShift) >= X27_RAMP_TABLE)
            s_rampShift++;
        for (uint8_t i = 0; i < X27_RAMP_TABLE; i++) {
            float v      = sqrt(v0 * v0 + 2 * a * (float)((uint16_t)i << s_rampShift));
            s_rampInc[i] = v >= vMax ? 0xFFFF : (uint16_t)(v / vMax * 65535.0f);
        }
    }

    // the table changed under any needle that is moving, restart their ramps
    for (uint8_t b = 0; b < s_numBoards; b++)
        for (uint8_t n = 0; n < X27_MOTORS; n++)
            s_boards[b]->_motors[n].ramp = 0;

    if (s_numBoards > 0) {
        timerStart(1000000UL / ((uint32_t)s_speedDeg * X27_STEPS_PER_DEG));
        timerUnmask();
    }
}

// ---------- commands from the Connector ----------
void PoStepVID6606::set(int16_t messageID, char *setPoint)
{
    if (!_initialised)
        return;

    if (messageID >= X27_MSG_DEG_FIRST && messageID < X27_MSG_DEG_FIRST + X27_MOTORS) {
        setTarget(messageID - X27_MSG_DEG_FIRST, degToSteps(setPoint));
        return;
    }
    if (messageID >= X27_MSG_RAW_FIRST && messageID < X27_MSG_RAW_FIRST + X27_MOTORS) {
        setTarget(messageID - X27_MSG_RAW_FIRST, (int16_t)constrain(atol(setPoint), 0L, (long)X27_MAX_POS));
        return;
    }

    int32_t value = atol(setPoint);
    switch (messageID) {
    case X27_MSG_STOP:
        // Connector stopped: needles drop to zero like a gauge losing power
        for (uint8_t n = 0; n < X27_MOTORS; n++)
            setTarget(n, 0);
        break;
    case X27_MSG_POWERSAVE:
        // targets are kept, so the needles return when power saving ends
        _parked = (value == 1);
        _active = true;
        break;
    case X27_MSG_HOME:
        if (value == 0)
            homeAll();
        else if (value >= 1 && value <= X27_MOTORS)
            home(value - 1);
        break;
    case X27_MSG_SPEED:
        if (value > 0) configureMotion(value, s_accelDeg);
        break;
    case X27_MSG_ACCEL:
        if (value >= 0) configureMotion(s_speedDeg, (uint16_t)min(value, (int32_t)X27_MAX_ACCEL));
        break;
    default:
        break;
    }
}

void PoStepVID6606::setTarget(uint8_t motor, int16_t steps)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        _motors[motor].target = steps;
        _active               = true;
    }
}

// Drive toward the zero stop further than the full sweep, then call that zero.
// A target set while homing is kept and the needle goes there afterwards.
void PoStepVID6606::home(uint8_t motor)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        Motor &m = _motors[motor];
        m.pos    = X27_HOME_OVERTRAVEL;
        m.dir    = 0;
        m.ramp   = 0;
        m.homing = true;
        _active  = true;
    }
}

void PoStepVID6606::homeAll()
{
    for (uint8_t n = 0; n < X27_MOTORS; n++)
        home(n);
}

// "90", "90.5" or "90,5" -> microsteps, clamped to the sweep. Avoids atof().
int16_t PoStepVID6606::degToSteps(const char *s)
{
    while (*s == ' ')
        s++;
    bool negative = (*s == '-');
    if (*s == '-' || *s == '+')
        s++;

    int32_t whole = 0;
    for (; isdigit(*s); s++)
        if (whole < 1000) whole = whole * 10 + (*s - '0');

    int32_t frac = 0, scale = 1;
    if (*s == '.' || *s == ',')
        for (s++; isdigit(*s) && scale < 1000; s++) {
            frac = frac * 10 + (*s - '0');
            scale *= 10;
        }

    if (negative)
        return 0;
    int32_t steps = whole * X27_STEPS_PER_DEG + (frac * X27_STEPS_PER_DEG + scale / 2) / scale;
    return (int16_t)min(steps, (int32_t)X27_MAX_POS);
}

// ---------- motion ----------
void PoStepVID6606::update()
{
#ifdef X27_NO_TIMER
    // every board gets called, but the first one in a period ticks them all
    uint32_t now = micros();
    if (now - s_lastTickUs < s_tickUs)
        return;
    s_lastTickUs = now;
    tickAll();
#endif
}

void PoStepVID6606::tickAll()
{
    for (uint8_t i = 0; i < s_numBoards; i++)
        s_boards[i]->tick();
}

void PoStepVID6606::tick()
{
    if (!_active)
        return;

    uint16_t w        = _word;
    uint16_t stepBits = 0;
    uint16_t stepBit  = 1; // word layout: bit 2n = STEP, bit 2n+1 = DIR of motor n
    bool     busy     = false;
    bool     parked   = _parked;
    Motor   *m        = _motors;

    for (uint8_t n = 0; n < X27_MOTORS; n++, m++, stepBit <<= 2) {
        int16_t delta = ((m->homing || parked) ? 0 : m->target) - m->pos;
        if (m->dir == 0) {
            if (delta == 0) continue;
            m->dir   = delta > 0 ? 1 : -1;
            m->ramp  = 0;
            m->phase = 0;
        }
        busy = true;

        uint16_t inc = s_homeInc;
        if (!m->homing) {
            uint16_t idx = m->ramp >> s_rampShift;
            inc          = s_rampInc[idx < X27_RAMP_TABLE ? idx : X27_RAMP_TABLE - 1];
        }
        uint16_t before = m->phase;
        m->phase += inc;
        if (m->phase >= before && inc != 0xFFFF) continue; // no step due in this tick

        int16_t dist = m->dir > 0 ? delta : -delta; // what is left in the current direction
        if (dist <= 0) {
            // arrived, or the target jumped behind the needle
            if (m->ramp == 0) {
                m->dir    = 0;
                m->homing = false; // pos == 0 here when homing
                continue;
            }
            m->ramp--; // too fast to stop or reverse, brake first
        } else if (!m->homing) {
            dist--;
            if (m->ramp > 0 && dist <= (int16_t)m->ramp)
                m->ramp--;
            else if (m->ramp < s_rampLen && dist > (int16_t)m->ramp + 1)
                m->ramp++;
        }

        m->pos += m->dir;
        w &= ~stepBit; // STEP low, the rising edge follows with the second word
        if ((m->dir < 0) == (X27_DIR_TOWARD_ZERO != 0))
            w |= stepBit << 1;
        else
            w &= ~(stepBit << 1);
        stepBits |= stepBit;
    }

    if (stepBits) {
        sendWord(w); // latch DIR + STEP low
        w |= stepBits;
        sendWord(w); // STEP high = one pulse for every motor in stepBits
        _word = w;
    }
    if (!busy)
        _active = false;
}

// Interrupts are only blocked per bit: long enough to keep the port
// read-modify-write safe against other ISRs, short enough for serial RX.
static inline void shiftByte(uint8_t data, volatile uint8_t *ser, uint8_t serMask, volatile uint8_t *sck, uint8_t sckMask)
{
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
        uint8_t sreg = SREG;
        cli();
        if (data & bit)
            *ser |= serMask;
        else
            *ser &= ~serMask;
        *sck |= sckMask;
        *sck &= ~sckMask;
        SREG = sreg;
    }
}

void PoStepVID6606::sendWord(uint16_t w)
{
    shiftByte(w >> 8, _serPort, _serMask, _sckPort, _sckMask); // MSB first
    shiftByte(w & 0xFF, _serPort, _serMask, _sckPort, _sckMask);

    uint8_t sreg = SREG;
    cli();
    *_rckPort |= _rckMask;
    *_rckPort &= ~_rckMask;
    SREG = sreg;
}
