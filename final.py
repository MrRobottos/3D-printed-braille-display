# 3 devices with 3 steppers + 3 buttons:
#  A = Bulb toggle (cloud)  -> Stepper A +90°
#  B = Plug toggle (cloud)  -> Stepper B +90°
#  C = Lock refresh trigger; Stepper C +90° only on real OPEN/CLOSED change
#
# Features:
#  - Loads config from .env
#  - Tuya connect + auto-reconnect on token invalid (code 1010)

import os, time, threading
import RPi.GPIO as GPIO
from gpiozero import Button
from dotenv import load_dotenv

# ---------- Config ----------
load_dotenv()

ENDPOINT      = os.getenv("TUYA_ENDPOINT", "https://openapi.tuyaeu.com")
ACCESS_ID     = os.getenv("TUYA_ACCESS_ID")
ACCESS_SECRET = os.getenv("TUYA_ACCESS_SECRET")
SCHEMA        = os.getenv("TUYA_SCHEMA", "smartlife")
USERNAME      = os.getenv("TUYA_USERNAME")
PASSWORD      = os.getenv("TUYA_PASSWORD")
COUNTRY_CODE  = os.getenv("TUYA_COUNTRY_CODE", "27")

BULB_ID  = os.getenv("TUYA_BULB_DEVICE_ID")
PLUG_ID  = os.getenv("TUYA_PLUG_DEVICE_ID")
LOCK_ID  = os.getenv("TUYA_LOCK_DEVICE_ID")

BULB_CODE = os.getenv("TUYA_BULB_SWITCH_CODE", "switch_led")
PLUG_CODE = os.getenv("TUYA_PLUG_SWITCH_CODE", "switch_1")

USE_TUYA = os.getenv("USE_TUYA", "1") == "1"

try:
    POLL_PERIOD_S = float(os.getenv("POLL_PERIOD_S", "1.0"))
    STEPS_90      = int(os.getenv("STEPS_90", "1024"))
    STEP_DELAY    = float(os.getenv("STEP_DELAY", "0.0028"))
except ValueError:
    POLL_PERIOD_S, STEPS_90, STEP_DELAY = 1.0, 1024, 0.0028

if USE_TUYA:
    missing = [k for k in ["TUYA_ACCESS_ID","TUYA_ACCESS_SECRET","TUYA_USERNAME","TUYA_PASSWORD","TUYA_COUNTRY_CODE","TUYA_SCHEMA"]
		   if not os.getenv(k)]
    if missing:
        raise RuntimeError(f"Missing env: {', '.join(missing)}")

# ---------- Pins (BCM) ----------
# Steppers (ULN2003 IN1..IN4)
A_PINS = [17, 18, 27, 22]  # Bulb prism
B_PINS = [23, 24, 25, 12]  # Plug prism
C_PINS = [16, 20, 21, 26]  # Lock prism

# Buttons
BTN_A = 4   # Bulb toggle
BTN_B = 6   # Plug toggle
BTN_C = 13  # Lock refresh

# ---------- Stepper driver ----------
SEQ = [
    [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
    [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1],
]

def setup_outputs(pins):
    for p in pins:
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, 0)

def release(pins):
    for p in pins:
        GPIO.output(p, 0)

class HalfStepper:
    def __init__(self, pins, delay=0.003):
        self.pins = pins
        self.delay = delay
        self.idx = 0
        self._lock = threading.Lock()
        setup_outputs(self.pins)

    def _phase(self, pattern):
        for pin, val in zip(self.pins, pattern):
            GPIO.output(pin, val)

    def step_n(self, steps):
        steps = abs(int(steps))
        with self._lock:
            for _ in range(steps):
                self.idx = (self.idx + 1) % 8
                self._phase(SEQ[self.idx])
                time.sleep(self.delay)
            release(self.pins)

# ---------- GPIO base ----------
GPIO.setmode(GPIO.BCM)
btn_a = Button(BTN_A, pull_up=True, bounce_time=0.15)
btn_b = Button(BTN_B, pull_up=True, bounce_time=0.15)
btn_c = Button(BTN_C, pull_up=True, bounce_time=0.15)

stepperA = HalfStepper(A_PINS, delay=STEP_DELAY)
stepperB = HalfStepper(B_PINS, delay=STEP_DELAY)
stepperC = HalfStepper(C_PINS, delay=STEP_DELAY)

# ---------- Tuya shared client ----------
api = None
api_ready = threading.Event()
api_lock  = threading.Lock()

def _ensure_import():
    try:
        from tuya_iot import TuyaOpenAPI  # noqa: F401
        return True
    except ImportError:
        print("Tuya SDK missing. Install: pip3 install --break-system-packages tuya-iot-py-sdk")
        return False

def _connect():
    from tuya_iot import TuyaOpenAPI
    _api = TuyaOpenAPI(ENDPOINT, ACCESS_ID, ACCESS_SECRET)
    resp = _api.connect(USERNAME, PASSWORD, COUNTRY_CODE, SCHEMA)
    if not resp or not resp.get("success"):
        raise RuntimeError(f"Cloud connect failed: {resp}")
    return _api

def init_tuya():
    global api
    if not USE_TUYA:
        print("Cloud disabled.")
        return
    if not _ensure_import():
        return
    print("Connecting to Tuya...")
    try:
        api = _connect()
        api_ready.set()
        print("Tuya connected.")
    except Exception as e:
        print(f"Cloud connect error: {e}")

def _api_post(path, payload):
    """POST with single reconnect on token invalid (1010)."""
    if not api_ready.wait(timeout=5):
        return {"success": False, "code": -1, "msg": "api not ready"}
    with api_lock:
        res = api.post(path, payload)
    if res and res.get("code") == 1010:
        # token invalid -> reconnect once
        try:
            with api_lock:
                print("Reconnecting to Tuya...")
                new = _connect()
                globals()["api"] = new
                res = api.post(path, payload)
        except Exception as e:
            return {"success": False, "code": 1010, "msg": f"reconnect failed: {e}"}
    return res

def _api_get(path):
    """GET with single reconnect on token invalid (1010)."""
    if not api_ready.wait(timeout=5):
        return {"success": False, "code": -1, "msg": "api not ready"}
    with api_lock:
        res = api.get(path)
    if res and res.get("code") == 1010:
        try:
            with api_lock:
                print("Reconnecting to Tuya...")
                new = _connect()
                globals()["api"] = new
                res = api.get(path)
        except Exception as e:
            return {"success": False, "code": 1010, "msg": f"reconnect failed: {e}"}
    return res

# ---------- Bulb ----------
bulb_on = False
def set_bulb(on: bool) -> bool:
    if not (USE_TUYA and BULB_ID):
        return False
    payload = {"commands": [{"code": BULB_CODE, "value": bool(on)}]}
    res = _api_post(f"/v1.0/devices/{BULB_ID}/commands", payload)
    return bool(res and res.get("success"))

def on_button_a():
    global bulb_on
    target = not bulb_on
    if set_bulb(target):
        bulb_on = target
        print(f"Bulb: {'ON' if bulb_on else 'OFF'}")
        stepperA.step_n(STEPS_90)
    else:
        print("Bulb: command failed")

btn_a.when_pressed = on_button_a

# ---------- Plug ----------
plug_on = False

def set_plug(on: bool) -> bool:
    if not (USE_TUYA and PLUG_ID):
        return False
    payload = {"commands": [{"code": PLUG_CODE, "value": bool(on)}]}
    res = _api_post(f"/v1.0/devices/{PLUG_ID}/commands", payload)
    return bool(res and res.get("success"))

def on_button_b():
    global plug_on
    target = not plug_on
    if set_plug(target):
        plug_on = target
        print(f"Plug: {'ON' if plug_on else 'OFF'}")
        stepperB.step_n(STEPS_90)
    else:
        print("Plug: command failed")

btn_b.when_pressed = on_button_b

# ---------- Lock (polling; no first-boot rotation) ----------
lock_last = None
lock_initialized = False
force_refresh = False  # set by Button C
def read_lock_state():
    """Return 'OPEN'/'CLOSED' or None if unknown."""
    if not (USE_TUYA and LOCK_ID):
        return None
    res = _api_get(f"/v1.0/devices/{LOCK_ID}/status")
    if not res or not res.get("success"):
        return None
    dps = {x["code"]: x["value"] for x in res["result"]}
    if "closed_opened" in dps:
        return "CLOSED" if dps["closed_opened"] == "closed" else "OPEN"
    if "lock_motor_state" in dps:
        return "CLOSED" if dps["lock_motor_state"] else "OPEN"
    return None

def on_button_c():
    global force_refresh
    force_refresh = True
    print("Lock: refresh")

btn_c.when_pressed = on_button_c

def lock_thread():
    global lock_last, lock_initialized, force_refresh
    # learn initial state without rotating
    while True:
        st = read_lock_state()
        if st is not None:
            lock_last = st
            lock_initialized = True
            print(f"Lock: {st}")
            break
        time.sleep(0.5)

    # now monitor for changes
    while True:
        st = read_lock_state()
        if st is not None and st != lock_last:
            lock_last = st
            print(f"Lock: {st}")
            stepperC.step_n(STEPS_90)
        # poll cadence with quick break on refresh
        for _ in range(int(POLL_PERIOD_S * 10)):
            if force_refresh:
                force_refresh = False
                break
            time.sleep(0.1)

# ---------- Main ----------
print("Starting...")
t_cloud = threading.Thread(target=init_tuya, daemon=True)
t_cloud.start()

t_lock = threading.Thread(target=lock_thread, daemon=True)
t_lock.start()

try:
    while True:
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    release(A_PINS); release(B_PINS); release(C_PINS)
    GPIO.cleanup()
    print("Exit.")