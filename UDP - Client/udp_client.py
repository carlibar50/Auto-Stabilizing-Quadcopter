import socket
import struct
import sys
import threading
import time

DRONE_IP   = "172.20.10.8"
DRONE_PORT = 4210
SEND_HZ    = 50

THROTTLE_STEP  = 50
ANGLE_STEP     = 200
YAW_STEP       = 300

KP_STEP = 0.1
KI_STEP = 0.005
KD_STEP = 0.01

THROTTLE_MIN   = 0
THROTTLE_MAX   = 1500
ANGLE_MAX      = 1500
YAW_MAX        = 3000

class State:
    def __init__(self):
        self.throttle  = 0
        self.pitch     = 0
        self.roll      = 0
        self.yaw_rate  = 0
        self.kp        = 1.2
        self.ki        = 0.02
        self.kd        = 0.05
        self.lock      = threading.Lock()
        self.running   = True

state = State()

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / SEND_HZ
    while state.running:
        with state.lock:
            pkt = struct.pack('<Hhhhfff',
                state.throttle,
                state.pitch,
                state.roll,
                state.yaw_rate,
                state.kp,
                state.ki,
                state.kd
            )
        try:
            sock.sendto(pkt, (DRONE_IP, DRONE_PORT))
        except Exception:
            pass
        time.sleep(interval)
    sock.close()

def print_status():
    with state.lock:
        thr = state.throttle
        p   = state.pitch / 100
        r   = state.roll / 100
        y   = state.yaw_rate / 100
        kp  = state.kp
        ki  = state.ki
        kd  = state.kd
    print(f"\rTHR={thr:4d} P={p:+6.1f} R={r:+6.1f} Y={y:+6.1f} | Kp={kp:.2f} Ki={ki:.3f} Kd={kd:.2f}  ", end="", flush=True)

if sys.platform == "win32":
    import msvcrt
    def read_key():
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ('\x00', '\xe0'):
                ch2 = msvcrt.getwch()
                return {
                    'H': 'UP', 'P': 'DOWN',
                    'K': 'LEFT', 'M': 'RIGHT',
                }.get(ch2, None)
            return ch.upper()
        return None
else:
    import tty, termios, select
    _fd = sys.stdin.fileno()
    _old = termios.tcgetattr(_fd)
    tty.setraw(_fd)
    def read_key():
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch2 = sys.stdin.read(2)
                return {
                    '[A': 'UP', '[B': 'DOWN',
                    '[D': 'LEFT', '[C': 'RIGHT',
                }.get(ch2, None)
            return ch.upper()
        return None
    import atexit
    atexit.register(lambda: termios.tcsetattr(_fd, termios.TCSADRAIN, _old))

def main():
    print("UDP Client - Tuning Edition")
    print("Target:", DRONE_IP)
    print("W/S: Throttle | ARROWS: Pitch/Roll | A/D: Yaw")
    print("U/J: Kp +/- | I/K: Ki +/- | O/L: Kd +/-")
    print("SPACE: STOP | Q: Quit")
    
    sender = threading.Thread(target=send_loop, daemon=True)
    sender.start()
    
    try:
        while True:
            key = read_key()
            if key is None:
                time.sleep(0.02)
                continue
            with state.lock:
                if key == 'Q':
                    state.throttle = 0
                    state.running  = False
                    break
                elif key == ' ':
                    state.throttle = 0
                    state.pitch    = 0
                    state.roll     = 0
                    state.yaw_rate = 0
                elif key == 'W':
                    state.throttle = clamp(state.throttle + THROTTLE_STEP, THROTTLE_MIN, THROTTLE_MAX)
                elif key == 'S':
                    state.throttle = clamp(state.throttle - THROTTLE_STEP, THROTTLE_MIN, THROTTLE_MAX)
                elif key == 'UP':
                    state.pitch = clamp(state.pitch + ANGLE_STEP, -ANGLE_MAX, ANGLE_MAX)
                elif key == 'DOWN':
                    state.pitch = clamp(state.pitch - ANGLE_STEP, -ANGLE_MAX, ANGLE_MAX)
                elif key == 'RIGHT':
                    state.roll = clamp(state.roll + ANGLE_STEP, -ANGLE_MAX, ANGLE_MAX)
                elif key == 'LEFT':
                    state.roll = clamp(state.roll - ANGLE_STEP, -ANGLE_MAX, ANGLE_MAX)
                elif key == 'D':
                    state.yaw_rate = clamp(state.yaw_rate + YAW_STEP, -YAW_MAX, YAW_MAX)
                elif key == 'A':
                    state.yaw_rate = clamp(state.yaw_rate - YAW_STEP, -YAW_MAX, YAW_MAX)
                elif key == 'U':
                    state.kp = round(state.kp + KP_STEP, 2)
                elif key == 'J':
                    state.kp = round(max(0.0, state.kp - KP_STEP), 2)
                elif key == 'I':
                    state.ki = round(state.ki + KI_STEP, 3)
                elif key == 'K':
                    state.ki = round(max(0.0, state.ki - KI_STEP), 3)
                elif key == 'O':
                    state.kd = round(state.kd + KD_STEP, 2)
                elif key == 'L':
                    state.kd = round(max(0.0, state.kd - KD_STEP), 2)
            print_status()
    except KeyboardInterrupt:
        with state.lock:
            state.throttle = 0
            state.running  = False

if __name__ == "__main__":
    main()