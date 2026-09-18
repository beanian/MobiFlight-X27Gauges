# MobiFlight X27 Gauges

MobiFlight custom firmware for the **Arduino Mega** that drives X27.168 gauge steppers through
**PoLabs PoStepVID6606** boards (two VID6606 chips behind a 16 bit shift register, 8 motors per board).

It is the normal MobiFlight Mega firmware (core 3.1.4) plus one community device, so buttons, encoders,
LEDs, displays etc. on the same Mega keep working. The sim side (variables, scaling, non-linear dials)
is configured in MobiFlight Connector like any other output. This replaces the standalone
[x27GaugeManager](https://github.com/beanian/x27GaugeManager) bridge + Nano firmware.

Built from the [MobiFlight CommunityTemplate](https://github.com/MobiFlight/CommunityTemplate).

## Install

```powershell
pip install platformio      # once
.\install.ps1               # builds and copies to %LOCALAPPDATA%\MobiFlight\MobiFlight Connector\Community\X27Gauges
```

1. **Back up first**: in the Connector open *Extras > Settings > MobiFlight Modules*, right click the Mega and
   save its config (`.mfmc`). The device config lives in EEPROM and normally survives a firmware change, but
   the board type changes, so have the backup.
2. Flash the Mega. The Connector only offers other firmware types for a board without MobiFlight firmware, so
   on an already flashed Mega either
   - close the Connector and upload from PlatformIO, which leaves the EEPROM config alone:
     `python -m platformio run -e x27gauges_mega -t upload --upload-port COM4`, or
   - restart the Connector, right click the Mega > *Reset board* (erases firmware **and** config), then
     right click > *Upload firmware* > **X27 Gauges Mega** and load the saved `.mfmc` again.
3. *Add device > Custom Device > PoStepVID6606*. Pick the three pins wired to the board's **SER**, **SCK** and
   **RCK**. Add a second device with three other pins for a second board.
4. Upload the config. All needles home against the zero stop (~4 s), which also happens on every power up
   and config upload.

To go back, flash the stock *MobiFlight Mega* firmware from the same menu.

### Wiring

| PoStep | Mega |
|---|---|
| SER | any free digital pin |
| SCK | any free digital pin |
| RCK | any free digital pin |
| GND | GND |
| 12-24 V | separate supply (not the 5 V rail) |

Stepping is driven by **Timer4**, so **PWM on D6, D7 and D8 stops working** (they still work as plain
on/off pins and as PoStep bus pins). Pick another timer with `-DX27_TIMER=` if you need those, see
[X27Gauges_platformio.ini](X27Gauges/X27Gauges_platformio.ini): Timer1 = D11/D12, Timer3 = D2/D3/D5,
Timer5 = D44-D46 (Timer5 is used by MobiFlight servos).

## Using it in a config

In an output config choose *Display type: Custom Device*, the PoStep device, and one of these message types:

| Id | Message | Value |
|---|---|---|
| 1-8 | Motor *n* - angle | Needle angle in degrees from the zero stop, `0`-`315`, decimals allowed (`90.5`). Resolution is 1/12 degree. |
| 11-18 | Motor *n* - raw microsteps | `0`-`3780` (12 per degree). For calibration and testing. |
| 20 | Home | `0` = all motors, `1`-`8` = one motor. Runs whenever the message is sent. |
| 21 | Max speed | deg/s, `30`-`400`. Shared by all boards. |
| 22 | Acceleration | deg/s^2, `0` = no ramp. Shared by all boards. |

*Motor n* is the label printed on the PoStep board (`Motor1`..`Motor8`), **not** the terminal position - the
screw terminals are pair-swapped along the strip (terminals 1-4 are Motor 2, 5-8 are Motor 1, and so on).

Map the sim value to degrees with the Connector's **Interpolation** modifier: put the needle on a few dial
marks using the raw or angle message in test mode, note the angles, and enter them as interpolation points.
That replaces the old manager's calibration table.

Out of range values are clamped. When the Connector stops, all needles return to zero; in power saving mode
they park at zero and come back afterwards.

### Speed and acceleration

The device's config string in the Modules dialog is `<max speed deg/s>|<acceleration deg/s^2>`, default
`280|3000`. Both are global (all boards share one timer; the last board in the config wins). `280|0`
reproduces `postep_gauges_fast.ino` exactly: fixed 297 us per microstep with no ramp. With the default ramp a
full sweep takes 1.19 s instead of 1.12 s, but needles start and stop at 60 deg/s, which helps a lot with
the low torque of a VID6606 at ~20 mA. Message types 21/22 change the same values live for tuning.

## How it works

- [PoStepVID6606.cpp](X27Gauges/PoStepVID6606.cpp) - one instance per board. A timer interrupt fires once per
  microstep period at max speed. Per motor a DDS accumulator decides whether to step in this tick, a
  "steps needed to brake" counter indexes a precomputed speed table for the ramp (no maths in the ISR).
  All motors of a board that step in the same tick share one pair of shift register writes
  (DIR + STEP low, then STEP high), about 60 us per board. Interrupts are only blocked per bit (~1.3 us), so serial
  traffic is not disturbed.
- Homing is non-blocking (it runs from the same interrupt), so the Connector handshake is not held up.
  A target received while homing is kept and the needle goes there afterwards.
- There is no position feedback. A stalled needle still counts as "arrived"; send *Home* for that motor.
- [MFCustomDevice.cpp](X27Gauges/MFCustomDevice.cpp) - MobiFlight glue: reads pins and config string from EEPROM.
- [tools/sim_motion.py](tools/sim_motion.py) - host-side model of the stepping logic with checks for exact
  arrival, staying on the scale while being retargeted, and homing. Run it after changing `tick()`.

Build options (`-DX27_NO_TIMER` to step from `loop()` instead of a timer, direction polarity, default
speeds) are documented in [X27Gauges_platformio.ini](X27Gauges/X27Gauges_platformio.ini).

## Status

Compiles against core firmware 3.1.4 and the motion logic passes the host-side simulation.
Bench-tested on 2026-09-18 on an Arduino Mega with one PoStepVID6606 board (homing and needle
positioning from MobiFlight Connector). A second board and all 16 needles moving at once are untested.
