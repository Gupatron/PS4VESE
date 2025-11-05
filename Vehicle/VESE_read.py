#!/usr/bin/env python3
import time, glob, serial, socket, sys, threading, math
from pyvesc import SetDutyCycle, encode

# --- Config ---
BAUD = 115200
POLE_PAIRS = 23.65
LISTEN_IP = "0.0.0.0"
PORT = 5005
SAFETY_TIMEOUT = 0.5

# --- Throttle limiting ---
MAX_DUTY = 0.1        # hard cap on throttle: [-0.30, +0.30]
MAX_STEP = 0.05       # max duty *change* per loop tick (smooth ramp)
# OPTIONAL: enable a soft RPM cap by uncommenting these lines in the main loop
# MAX_RPM = 1500

# --- Kalman Filter ---
class KalmanFilter:
    def __init__(self, x0=0.0, P0=100.0, Q=0.1, R=10.0):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.R = R
    def predict(self):
        self.P += self.Q
    def update(self, z):
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P = (1 - K) * self.P

# --- Helper: CRC + packing ---
def _crc16_xmodem(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def pack_comm(payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("Use long frame for payload >255")
    start = b"\x02"
    length = bytes([len(payload)])
    crc = _crc16_xmodem(payload)
    crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    end = b"\x03"
    return start + length + payload + crc_bytes + end

# --- Port discovery ---
def pick_port():
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        raise SystemExit("No VESC serial ports found.")
    return ports[0]

# --- Telemetry thread ---
def telemetry_loop(ser, kf, stop_flag, shared):
    ser.timeout = 0  # non-blocking
    last_req = 0.0
    buf = b""
    while not stop_flag[0]:
        now = time.time()
        if now - last_req > 0.1:          # request at ~10 Hz
            ser.write(pack_comm(b"\x04"))
            ser.flush()
            last_req = now

        n = ser.in_waiting
        if n > 0:
            buf += ser.read(n)

            # Try to parse when we likely have a full frame.
            # (This is a minimal heuristic; a robust parser would frame by CRC)
            if len(buf) >= 32:
                try:
                    rpm_bytes = buf[25:29]
                    rpm_val = int.from_bytes(rpm_bytes, 'big', signed=True)
                    mrpm = rpm_val / POLE_PAIRS
                    kf.predict()
                    kf.update(mrpm)
                    shared["rpm"] = kf.x
                except Exception:
                    pass
                buf = b""  # clear buffer and continue
        time.sleep(0.002)

# --- Main loop ---
def main():
    port = pick_port()
    print(f"Opening VESC on {port} @ {BAUD}")
    try:
        ser = serial.Serial(port, BAUD, timeout=0)
    except Exception as e:
        print(f"Serial open failed: {e}")
        sys.exit(1)

    kf = KalmanFilter()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, PORT))
    sock.setblocking(0)
    print(f"Listening for controller commands on {LISTEN_IP}:{PORT}")

    current_input_duty = 0.0   # raw command from network
    applied_duty = 0.0         # what we actually send after limits
    last_packet_time = time.time()
    shared = {"rpm": 0.0}
    stop_flag = [False]

    t = threading.Thread(target=telemetry_loop, args=(ser, kf, stop_flag, shared), daemon=True)
    t.start()

    try:
        while True:
            # --- Receive network commands ---
            try:
                data, _ = sock.recvfrom(1024)
                current_input_duty = float(data.decode('utf-8'))
                last_packet_time = time.time()
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"Packet error: {e}")

            # --- Safety timeout ---
            if time.time() - last_packet_time > SAFETY_TIMEOUT:
                if applied_duty != 0.0:
                    print("\nSAFETY TIMEOUT — zeroing duty cycle.")
                current_input_duty = 0.0

            # --- Apply duty limit and ramp (smooth) ---
            # 1) Hard cap to [-MAX_DUTY, +MAX_DUTY]
            target = max(min(current_input_duty, MAX_DUTY), -MAX_DUTY)

            # 2) Rate limit: bound how fast applied_duty can change per tick
            delta = target - applied_duty
            if abs(delta) > MAX_STEP:
                applied_duty += math.copysign(MAX_STEP, delta)
            else:
                applied_duty = target

            # OPTIONAL: Soft RPM limiter (uncomment to use)
            # if abs(shared["rpm"]) > MAX_RPM:
            #     # Nudge duty down gently to pull RPM under cap
            #     applied_duty *= 0.98

            # --- Send to VESC ---
            duty_cmd = int(applied_duty * 100000)   # pyvesc expects scaled int
            ser.write(encode(SetDutyCycle(duty_cmd)))
            ser.flush()

            # --- Print telemetry at ~10 Hz ---
            if int(time.time() * 10) % 10 == 0:
                rpm = shared["rpm"]
                print(f"Duty(in): {current_input_duty:6.3f} | Duty(out): {applied_duty:6.3f} | RPM: {rpm:9.1f}", end="\r")

            time.sleep(0.01)  # ~100 Hz control loop

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stop_flag[0] = True
        t.join(timeout=0.2)
        print("Setting duty cycle to 0.")
        ser.write(encode(SetDutyCycle(0)))
        ser.flush()
        ser.close()
        sock.close()
        print("Jetson script terminated.")

if __name__ == "__main__":
    main()

