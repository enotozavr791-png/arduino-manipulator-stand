import socket
import math
import threading
import time
import serial
import pybullet as p
import pybullet_data

# --- Константы ---
ARDUINO_PORT = 'COM4'
BASE_TOP = 0.09
L1, L2 = 0.155, 0.215

SPEED_SLOW, SPEED_FAST = 40.0, 120.0
DEADZONE, CURVE, SMOOTHING = 0.15, 3.0, 0.10

# --- Arduino ---
try:
    arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
    time.sleep(2)
    print(f"[*] Arduino подключена на {ARDUINO_PORT}")
    arduino.write(b"90,0,90\n")
except Exception as e:
    print(f"[!] Arduino не подключена: {e}")
    arduino = None

# --- Контроллер ---
try:
    from inputs import get_gamepad
except Exception:
    get_gamepad = None
    print("[!] inputs не установлена")

# --- Состояние ---
_lock = threading.Lock()
_angles = [90.0, 0.0, 90.0]       # Текущие (сглаженные)
_targets = [90.0, 0.0, 90.0]      # Целевые (от стика)
_max_speed = SPEED_SLOW
_y_pressed = False

def set_targets(a0, a1, a2):
    with _lock:
        _targets[0], _targets[1], _targets[2] = a0, a1, a2

def update_angles():
    with _lock:
        for i in range(3):
            _angles[i] += (_targets[i] - _angles[i]) * SMOOTHING
        return tuple(_angles)

# --- PyBullet Инициализация ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setPhysicsEngineParameter(fixedTimeStep=1/240, numSubSteps=4)
p.resetDebugVisualizerCamera(2.5, 45, -30, [0, 0, 0.4])
p.loadURDF("plane.urdf")

def _body(shape, color, pos, half=None):
    if shape == 'cyl':
        c = p.createCollisionShape(p.GEOM_CYLINDER, radius=half[0], height=half[1])
        v = p.createVisualShape(p.GEOM_CYLINDER, radius=half[0], length=half[1], rgbaColor=color)
    elif shape == 'box':
        c = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        v = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    else:
        c, v = -1, p.createVisualShape(p.GEOM_SPHERE, radius=half[0], rgbaColor=color)
    return p.createMultiBody(0, c, v, pos)

# --- Манипулятор ---
_body('cyl', [0.3, 0.3, 0.3, 1], [0, 0, BASE_TOP/2], half=(0.10, BASE_TOP/2))
arm = [
    _body('box', [0.2, 0.6, 1.0, 1], [L1 / 2, 0, BASE_TOP], half=[L1 / 2, 0.02, 0.02]),
    _body('box', [0.2, 1.0, 0.4, 1], [L1 + L2 / 2, 0, BASE_TOP], half=[L2 / 2, 0.02, 0.02]),
]
tip_marker = _body('sphere', [1, 0, 0, 1], [L1 + L2, 0, BASE_TOP], half=(0.015,))

# --- Вспомогательные функции ---
def curve(v):
    return v if CURVE == 1.0 else math.copysign(abs(v) ** CURVE, v)

def send_to_arduino(a0, a1, a2):
    if arduino:
        try:
            arduino.write(f"{int(a0)},{int(a1)},{int(a2)}\n".encode())
        except Exception as e:
            print(f"[!] Serial write error: {e}")

def update_simulation(a0, a1, a2):
    t0 = math.radians(a0 - 90)
    t1 = math.radians(90 - a1)
    t2 = -math.radians(90 - a2) - math.pi/2  # Инверсия локтя в симуляции

    c0, s0 = math.cos(t0), math.sin(t0)
    c1, s1 = math.cos(t1), math.sin(t1)
    
    d1 = (c1 * c0, c1 * s0, s1)
    j2 = [d1[i] * L1 for i in range(3)]
    j2[2] += BASE_TOP
    c1p = [d1[i] * L1 / 2 for i in range(3)]
    c1p[2] += BASE_TOP
    
    p.resetBasePositionAndOrientation(arm[0], c1p, p.getQuaternionFromEuler([0, -t1, t0]))

    t12 = t1 + t2
    c12, s12 = math.cos(t12), math.sin(t12)
    d2 = (c12 * c0, c12 * s0, s12)
    c2p = [j2[i] + d2[i] * L2 / 2 for i in range(3)]
    p.resetBasePositionAndOrientation(arm[1], c2p, p.getQuaternionFromEuler([0, -t12, t0]))

    p.resetBasePositionAndOrientation(tip_marker, [j2[i] + d2[i] * L2 for i in range(3)], [0, 0, 0, 1])

# --- Потоки ---
_axes = {'ABS_X': 0.0, 'ABS_Y': 0.0, 'ABS_RY': 0.0}
_ax_lock = threading.Lock()

def controller_reader():
    global _max_speed, _y_pressed
    if not get_gamepad:
        return
    while True:
        try:
            events = get_gamepad()
        except Exception:
            time.sleep(0.05)
            continue
        with _ax_lock:
            for e in events:
                if e.code in _axes:
                    _axes[e.code] = e.state / 32768.0
                elif e.code == 'BTN_NORTH':
                    if e.state == 1 and not _y_pressed:
                        _y_pressed = True
                        _max_speed = SPEED_FAST if _max_speed == SPEED_SLOW else SPEED_SLOW
                        print(f"[*] Скорость: {'БЫСТРАЯ' if _max_speed == SPEED_FAST else 'МЕДЛЕННАЯ'}")
                    elif e.state == 0:
                        _y_pressed = False
        time.sleep(0.001)

def controller_motion():
    pt = dt0 = time.time()
    while True:
        now = time.time()
        dt = now - pt
        pt = now
        
        with _ax_lock:
            lx, ly, ry = _axes['ABS_X'], _axes['ABS_Y'], _axes['ABS_RY']

        lx = curve(lx) if abs(lx) > DEADZONE else 0.0
        ly = curve(ly) if abs(ly) > DEADZONE else 0.0
        ry = curve(ry) if abs(ry) > DEADZONE else 0.0

        if lx or ly or ry:
            with _lock:
                a0, a1, a2 = _targets
            
            a0 -= lx * _max_speed * dt
            a1 -= ly * _max_speed * dt
            a2 += ry * _max_speed * dt
            
            with _lock:
                _targets[0] = max(0, min(180, a0))
                _targets[1] = max(0, min(180, a1))
                _targets[2] = max(0, min(180, a2))

        if now - dt0 > 1.0:
            print(f"[dbg] Angles: {_angles[0]:.0f}, {_angles[1]:.0f}, {_angles[2]:.0f} (Speed: {'FAST' if _max_speed == SPEED_FAST else 'SLOW'})")
            dt0 = now
        time.sleep(0.01)

def phone_server():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', 5000))
    srv.listen(1)
    print(f"[*] TCP на {socket.gethostbyname(socket.gethostname())}:5000")
    
    def ik(px, py, pz):
        r = math.hypot(px, py)
        h = pz - BASE_TOP
        D = max(min(math.hypot(r, h), L1 + L2 - 1e-3), abs(L1 - L2) + 1e-3)
        ca = max(-1.0, min(1.0, (L1**2 + D**2 - L2**2) / (2 * L1 * D)))
        t1 = math.atan2(h, r) + math.acos(ca)
        cg = max(-1.0, min(1.0, (L1**2 + L2**2 - D**2) / (2 * L1 * L2)))
        t2 = math.acos(cg) - math.pi
        t0 = math.atan2(py, px)
        return 90 + math.degrees(t0), 90 - math.degrees(t1), 90 - math.degrees(t2)

    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            for line in conn.makefile('r'):
                pr = line.strip().split(',')
                if len(pr) == 4:
                    try:
                        set_targets(*ik(*map(float, pr[:4])))
                    except ValueError:
                        pass
        except Exception:
            pass
        finally:
            conn.close()

# --- Запуск ---
if __name__ == '__main__':
    if get_gamepad:
        threading.Thread(target=controller_reader, daemon=True).start()
    threading.Thread(target=controller_motion, daemon=True).start()
    threading.Thread(target=phone_server, daemon=True).start()
    
    try:
        while p.isConnected():
            a0, a1, a2 = update_angles()
            send_to_arduino(a0, a1, a2)
            update_simulation(a0, a1, a2)
            p.stepSimulation()
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        print("\n[*] Остановлено")
    finally:
        if arduino:
            arduino.close()
        if p.isConnected():
            p.disconnect()