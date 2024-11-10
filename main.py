import time
import board
import pwmio
import analogio
import adafruit_icm20x
import fusion
import math


FSR_MIN = 1000
FSR_MAX = 30000
FSR_RANGE = FSR_MAX - FSR_MIN
LED_MAX = 65535

RED_IDLE = int(0.2 * LED_MAX)
BLUE_IDLE = int(0.2 * LED_MAX)
GREEN_IDLE = int(0.2 * LED_MAX)
IDLE_FREQ = 0.3
TRIGGER_FREQ = 2

STATE_2_LENGTH = 50
STATE_3_LENGTH = 50
STATE_0_1_PITCH_THRESH = -10

JERK_THRESH = 2500
JERK_COOLDOWN = 50


red = pwmio.PWMOut(board.GP22, frequency=1000)
green = pwmio.PWMOut(board.GP28, frequency=1000)
blue = pwmio.PWMOut(board.GP27, frequency=1000)

fsr = analogio.AnalogIn(board.GP26)

i2c = board.STEMMA_I2C()
imu = adafruit_icm20x.ICM20948(i2c)
fuse = fusion.Fusion()


def getmag():
    return imu.magnetic

def update_fuse():
    fuse.update(imu.acceleration, imu.gyro, imu.magnetic)

def linear(t, start, end):
    p = (t - start[0]) / (end[0] - start[0])
    return int(p * (end[1] - start[1]) + start[1])

def sine(t, amp, freq, offset=0, phase=0):
    return int(amp*math.sin(2*math.pi*freq*t + phase) + offset)

def cycle_rgb(angle, amp):
    if angle < 60:
        red_val = amp
        green_val = linear(angle, (0, 0), (60, amp))
        blue_val  = 0
    elif angle < 120:
        red_val = linear(angle, (60, amp), (120, 0))
        green_val = amp
        blue_val = 0
    elif angle < 180:
        red_val = 0
        green_val = amp
        blue_val = linear(angle, (120, 0), (180, amp))
    elif angle < 240:
        red_val = 0
        green_val = linear(angle, (180, amp), (240, 0))
        blue_val = amp
    elif angle < 300:
        red_val = linear(angle, (240, 0), (300, amp))
        green_val = 0
        blue_val = amp
    else:
        red_val = amp
        green_val = 0
        blue_val = linear(angle, (300, amp), (360, 0))
    return red_val, green_val, blue_val

def apply_fsr(red_vals, green_vals, blue_vals):
    fsr_val = fsr.value
    if fsr_val >= FSR_MIN and fsr_val <= FSR_MAX:
        red_amp = linear(fsr_val, (FSR_MIN, red_vals[0]), (FSR_MAX, red_vals[1]))
        #green_amp = linear(fsr_val, (FSR_MIN, green_vals[0]), (FSR_MAX, green_vals[1]))
        #blue_amp = linear(fsr_val, (FSR_MIN, blue_vals[0]), (FSR_MAX, blue_vals[1]))
        angle = int(200*time.monotonic()) % 360
        red_val, green_val, blue_val = cycle_rgb(angle, red_amp)
        red.duty_cycle = red_val
        green.duty_cycle = green_val
        blue.duty_cycle = blue_val
    elif fsr_val < FSR_MIN:
        red.duty_cycle = red_vals[0]
        green.duty_cycle = green_vals[0]
        blue.duty_cycle = blue_vals[0]
    else:
        red.duty_cycle = LED_MAX
        green.duty_cycle = LED_MAX
        blue.duty_cycle = LED_MAX


print("Calibrating...")
start_time = time.time()
fuse.calibrate(getmag, lambda : time.time() - start_time >= 5, 0)
print("Done calibrating")

transition_count = 0
jerk_cooldown = 0
update_fuse()
state = 0 if fuse.pitch < STATE_0_1_PITCH_THRESH else 1

prev_accel = imu.acceleration
prev_state = state

while True:
    update_fuse()
    accel = imu.acceleration
    jerk = ((accel[0] - prev_accel[0]) / 0.02, (accel[1] - prev_accel[1]) / 0.02, (accel[2] - prev_accel[2]) / 0.02)
    jerk_mag = math.sqrt(jerk[0]**2 + jerk[1]**2 + jerk[2]**2)
    if state == 0:
        if fuse.pitch >= STATE_0_1_PITCH_THRESH:
            state = 2
            transition_count = 0
        elif jerk_mag > JERK_THRESH and jerk_cooldown == 0:
            state = 4
            jerk_cooldown = JERK_COOLDOWN
            prev_state = 0
        else:
            if jerk_cooldown > 0:
                jerk_cooldown -= 1
            blue_val = sine(time.monotonic(), BLUE_IDLE/2, IDLE_FREQ, BLUE_IDLE/2)
            apply_fsr((0, LED_MAX), (0, LED_MAX), (blue_val, LED_MAX))
    elif state == 1:
        if fuse.pitch < STATE_0_1_PITCH_THRESH:
            state = 3
            transition_count = 0
        elif jerk_mag > JERK_THRESH and jerk_cooldown == 0:
            print("jorkin it")
            print(jerk_mag)
            state = 4
            jerk_cooldown = JERK_COOLDOWN
            prev_state = 1
        else:
            if jerk_cooldown > 0:
                jerk_cooldown -= 1
            red_val = sine(time.monotonic(), RED_IDLE/2, IDLE_FREQ, RED_IDLE/2)
            apply_fsr((red_val, 40000), (0, 40000), (0, 40000))
    elif state == 2:
        if transition_count == STATE_2_LENGTH:
            state = 1
        else:
            transition_count += 1
            blue_val = linear(transition_count, (0, BLUE_IDLE), (STATE_2_LENGTH, 0))
            blue_val = sine(time.monotonic(), blue_val/2, 0.3, blue_val/2)
            red_val = linear(transition_count, (0, 0), (STATE_2_LENGTH, RED_IDLE))
            red_val = sine(time.monotonic(), red_val/2, 0.3, red_val/2)
            blue.duty_cycle = blue_val
            red.duty_cycle = red_val
            #apply_fsr((red_val, red_val), (0, LED_MAX), (blue_val, LED_MAX))
    elif state == 3:
        if transition_count == STATE_3_LENGTH:
            state = 0
        else:
            transition_count += 1
            blue_val = linear(transition_count, (0, 0), (STATE_2_LENGTH, BLUE_IDLE))
            blue_val = sine(time.monotonic(), blue_val/2, 0.3, blue_val/2)
            red_val = linear(transition_count, (0, RED_IDLE), (STATE_2_LENGTH, 0))
            red_val = sine(time.monotonic(), red_val/2, 0.3, red_val/2)
            blue.duty_cycle = blue_val
            red.duty_cycle = red_val
            #apply_fsr((red_val_val, LED_MAX), (0, LED_MAX), (blue_val, blue_val))
    elif state == 4:
        if jerk_cooldown > 0:
            jerk_cooldown -= 1
        if jerk_mag > JERK_THRESH and jerk_cooldown == 0:
            print("unjorkin it")
            print(jerk_mag)
            state = prev_state
            jerk_cooldown = JERK_COOLDOWN
        else:
            angle = int(360*time.monotonic()) % 360
            red_val, green_val, blue_val = cycle_rgb(angle, LED_MAX)
            red.duty_cycle = red_val
            green.duty_cycle = green_val
            blue.duty_cycle = blue_val
    prev_accel = accel
    time.sleep(0.02)
