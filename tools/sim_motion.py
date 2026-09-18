"""Host-side model of the stepping logic in X27Gauges/PoStepVID6606.cpp.

Mirrors configureMotion() and tick() line for line so the ramp/arrival logic can be
checked without hardware. Keep it in sync when changing the firmware.

    python tools/sim_motion.py
"""
import math
import random

STEPS_PER_DEG = 12
MAX_POS = 315 * STEPS_PER_DEG
HOME_OVERTRAVEL = MAX_POS + 400
RAMP_TABLE = 128
START_SPEED = 60
HOME_SPEED = 90


class Motion:
    def __init__(self, speed_deg=280, accel_deg=3000):
        self.speed_deg = max(30, min(400, speed_deg))
        self.accel_deg = min(accel_deg, 30000)
        v_max = self.speed_deg * STEPS_PER_DEG
        v0 = min(v_max, START_SPEED * STEPS_PER_DEG)
        a = self.accel_deg * STEPS_PER_DEG
        self.tick_us = 1_000_000 // v_max
        self.home_inc = int(min(65535.0, HOME_SPEED / self.speed_deg * 65535.0))
        self.ramp_shift = 0
        self.ramp_inc = [0xFFFF] * RAMP_TABLE
        if self.accel_deg == 0 or v0 >= v_max:
            self.ramp_len = 0
        else:
            self.ramp_len = int(min(60000.0, (v_max * v_max - v0 * v0) / (2 * a)))
            while (self.ramp_len >> self.ramp_shift) >= RAMP_TABLE:
                self.ramp_shift += 1
            for i in range(RAMP_TABLE):
                v = math.sqrt(v0 * v0 + 2 * a * (i << self.ramp_shift))
                self.ramp_inc[i] = 0xFFFF if v >= v_max else int(v / v_max * 65535.0)


class Motor:
    def __init__(self, motion):
        self.k = motion
        self.pos = self.target = self.phase = self.ramp = self.dir = 0
        self.homing = False
        self.step_ticks = []  # tick numbers at which a pulse was sent
        self.now = 0

    def home(self):
        self.pos, self.dir, self.ramp, self.homing = HOME_OVERTRAVEL, 0, 0, True

    def tick(self):
        """Returns True while the motor is busy."""
        k = self.k
        self.now += 1
        delta = (0 if self.homing else self.target) - self.pos
        if self.dir == 0:
            if delta == 0:
                return False
            self.dir = 1 if delta > 0 else -1
            self.ramp = 0
            self.phase = 0

        inc = k.home_inc
        if not self.homing:
            inc = k.ramp_inc[min(self.ramp >> k.ramp_shift, RAMP_TABLE - 1)]
        before = self.phase
        self.phase = (self.phase + inc) & 0xFFFF
        if self.phase >= before and inc != 0xFFFF:
            return True

        dist = delta if self.dir > 0 else -delta
        if dist <= 0:
            if self.ramp == 0:
                self.dir = 0
                self.homing = False
                return True
            self.ramp -= 1
        elif not self.homing:
            dist -= 1
            if self.ramp > 0 and dist <= self.ramp:
                self.ramp -= 1
            elif self.ramp < k.ramp_len and dist > self.ramp + 1:
                self.ramp += 1

        self.pos += self.dir
        self.step_ticks.append(self.now)
        return True

    def run_until_idle(self, limit=200_000):
        n = 0
        while self.tick():
            n += 1
            assert n < limit, "never arrived"
        return n


def check(cond, msg):
    if not cond:
        raise SystemExit("FAIL: " + msg)


def test_full_sweep(speed, accel):
    k = Motion(speed, accel)
    m = Motor(k)
    m.target = MAX_POS
    ticks = m.run_until_idle()
    check(m.pos == MAX_POS and m.ramp == 0, "sweep did not land on target")
    gaps = [b - a for a, b in zip(m.step_ticks, m.step_ticks[1:])]
    check(min(gaps) >= 1, "two pulses in one tick")
    secs = ticks * k.tick_us / 1e6
    peak = 1e6 / (min(gaps) * k.tick_us) / STEPS_PER_DEG
    first = 1e6 / (gaps[0] * k.tick_us) / STEPS_PER_DEG
    last = 1e6 / (gaps[-1] * k.tick_us) / STEPS_PER_DEG
    print(f"  sweep 315deg @ {speed}deg/s accel {accel}: {secs:.2f}s, tick {k.tick_us}us, "
          f"ramp {k.ramp_len} steps (shift {k.ramp_shift}), start {first:.0f} peak {peak:.0f} end {last:.0f} deg/s")
    if accel:
        check(first <= START_SPEED * 1.6 and last <= START_SPEED * 1.6, "ramp does not start/end slow")


def test_short_moves(k):
    for d in list(range(1, 40)) + [100, 149, 150, 151, 300, 301]:
        m = Motor(k)
        m.pos = 1000
        m.target = 1000 + d
        m.run_until_idle()
        check(m.pos == 1000 + d and m.ramp == 0, f"short move +{d}")
        m.target = 1000
        m.run_until_idle()
        check(m.pos == 1000 and m.ramp == 0, f"short move -{d}")


def test_homing(k):
    m = Motor(k)
    m.pos = 1234
    m.target = 600  # set while homing, must be kept
    m.home()
    ticks = 0
    while m.homing:
        m.tick()
        ticks += 1
        check(ticks < 100_000, "homing never finished")
    check(m.pos == 0, "homing did not end at zero")
    print(f"  homing takes {ticks * k.tick_us / 1e6:.2f}s")
    m.run_until_idle()
    check(m.pos == 600, "target set during homing was lost")


def test_random_retarget(k, seed):
    rnd = random.Random(seed)
    m = Motor(k)
    lo = hi = 0
    for _ in range(3000):
        m.target = rnd.choice([0, MAX_POS, rnd.randint(0, MAX_POS), max(0, min(MAX_POS, m.pos + rnd.randint(-30, 30)))])
        for _ in range(rnd.randint(1, 1500)):
            m.tick()
            lo, hi = min(lo, m.pos), max(hi, m.pos)
    check(lo >= 0 and hi <= MAX_POS, f"needle left the scale while retargeting: {lo}..{hi}")
    m.run_until_idle()
    check(m.pos == m.target and m.ramp == 0, "did not settle after retargeting")


if __name__ == "__main__":
    for speed, accel in [(280, 3000), (280, 0), (400, 3000), (400, 300), (100, 1000), (30, 3000), (280, 30000)]:
        test_full_sweep(speed, accel)
    for speed, accel in [(280, 3000), (280, 0), (400, 300), (280, 30000)]:
        k = Motion(speed, accel)
        test_short_moves(k)
        test_homing(k)
        for seed in range(5):
            test_random_retarget(k, seed)
    print("all motion checks passed")
