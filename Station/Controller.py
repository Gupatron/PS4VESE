#!/usr/bin/env python3
import socket, time, statistics, math
import pygame

# --- Network Config ---
JETSON_IP = "192.168.XX.XX"   # <- set your Jetson's IP
PORT = 5005
SEND_INTERVAL = 0.01  # 100 Hz

# --- Control Config ---
AXIS_L2 = 2   # your L2 index
AXIS_R2 = 5   # your R2 index

# Per-trigger deadzone after mapping to [0..1]
TRIGGER_DEADZONE = 0.08
# Final center deadband on duty: [-DB..+DB] -> 0
CENTER_DEADBAND = 0.06
# Optional smoothing (EMA) on the final duty
SMOOTHING_ALPHA = 0.20  # 0 = off; 0.1–0.3 = light smoothing

# Output duty mapping
MIN_OUTPUT_DUTY = 0.02
MAX_OUTPUT_DUTY = 0.4

def detect_range_mode(js, axis_index, samples=200, hold_time=0.5):
    """
    Returns '01' if the axis looks like 0..1 (rest ~0),
            'pm' if it looks like -1..+1 (rest ~-1).
    """
    vals = []
    t_end = time.time() + hold_time
    while time.time() < t_end:
        pygame.event.pump()
        if axis_index < js.get_numaxes():
            vals.append(js.get_axis(axis_index))
        time.sleep(max(hold_time / samples, 0.001))
    if not vals:
        return 'pm'  # default safe assumption

    med = statistics.median(vals)
    # Heuristic: if median near 0, call it 0..1; if near -1, call it -1..+1
    return '01' if med > -0.5 else 'pm'

def map_trigger(val, mode):
    """
    Map raw trigger to [0..1] regardless of driver mode.
    mode 'pm' => -1..+1 (rest ~-1).  (val + 1)/2
    mode '01' =>  0..1   (rest ~ 0).  val (clamped)
    """
    if mode == 'pm':
        v = (val + 1.0) * 0.5
    else:
        v = val
    if v < 0.0: v = 0.0
    if v > 1.0: v = 1.0
    return v

def apply_deadzone(x, dz):
    return 0.0 if x < dz else x

def apply_center_deadband(x, db):
    return 0.0 if abs(x) < db else x

def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller detected.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()} (axes={js.get_numaxes()})")
    print(f"Using triggers R2 axis={AXIS_R2}, L2 axis={AXIS_L2}")
    print("Please release both triggers for 0.5s while I detect their ranges...")

    # Auto-detect range mode for each trigger (rest state)
    r2_mode = detect_range_mode(js, AXIS_R2)
    l2_mode = detect_range_mode(js, AXIS_L2)
    print(f"Detected R2 mode: {'0..1' if r2_mode=='01' else '-1..+1'}")
    print(f"Detected L2 mode: {'0..1' if l2_mode=='01' else '-1..+1'}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_send = 0.0
    duty_out = 0.0  # smoothed output

    try:
        while True:
            pygame.event.pump()

            num_axes = js.get_numaxes()
            r2_raw = js.get_axis(AXIS_R2) if AXIS_R2 < num_axes else 0.0
            l2_raw = js.get_axis(AXIS_L2) if AXIS_L2 < num_axes else 0.0

            # Map to [0..1] robustly for each trigger
            r2 = map_trigger(r2_raw, r2_mode)
            l2 = map_trigger(l2_raw, l2_mode)

            # Per-trigger deadzones remove tiny drift at rest
            r2 = apply_deadzone(r2, TRIGGER_DEADZONE)
            l2 = apply_deadzone(l2, TRIGGER_DEADZONE)

            # Bidirectional duty: forward = R2, reverse = L2
            duty = r2 - l2  # in [-1..+1]

            # Center deadband prevents tiny movement
            duty = apply_center_deadband(duty, CENTER_DEADBAND)

            # Optional smoothing
            if SMOOTHING_ALPHA > 0.0:
                duty_out = (1.0 - SMOOTHING_ALPHA) * duty_out + SMOOTHING_ALPHA * duty
            else:
                duty_out = duty

            # Map to min and max output duty
            if abs(duty_out) > 0:
                sign = math.copysign(1.0, duty_out)
                mag = abs(duty_out)
                scaled_mag = MIN_OUTPUT_DUTY + (MAX_OUTPUT_DUTY - MIN_OUTPUT_DUTY) * (mag - CENTER_DEADBAND) / (1.0 - CENTER_DEADBAND)
                duty_out = sign * scaled_mag

            # Clamp for safety before sending
            if duty_out > MAX_OUTPUT_DUTY: duty_out = MAX_OUTPUT_DUTY
            if duty_out < -MAX_OUTPUT_DUTY: duty_out = -MAX_OUTPUT_DUTY

            now = time.time()
            if now - last_send >= SEND_INTERVAL:
                sock.sendto(str(duty_out).encode("utf-8"), (JETSON_IP, PORT))
                last_send = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nExiting controller script.")
    finally:
        sock.close()
        pygame.quit()

if __name__ == "__main__":
    main()