import time
import json
import socket
import network
import gc
from machine import Pin, PWM, SPI, I2C
from esp32 import NVS
from config import WIFI_SSID, WIFI_PASSWORD, CONTROL_KEY
from auto_network import AutoNetwork

PIN_SERVO = 2
PIN_BUZZER = 15
PIN_BUTTON = 34
ROW_PINS = [13, 12, 14, 27]
COL_PINS = [26, 25, 33, 32]
KEYPAD_MAP = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"]
]
LCD_SDA, LCD_SCL, LCD_ADDR = 21, 22, 0x27
SCK_PIN, MISO_PIN, MOSI_PIN, RST_PIN, SS_PIN = 18, 19, 23, 4, 5
DEFAULT_PASSWORD = "66"
DEFAULT_RFIDS = ["5A231907", "083E3846", "8802165E"]
SERVO_LOCK_ANGLE = 90
SERVO_OPEN_ANGLE = 0
SERVO_RELEASE = True
SERVO_MOVE_MS = 500
DOOR_OPEN_MS = 5000
PASSWORD_TIMEOUT_MS = 10000
RFID_SCAN_MS = 80
MAX_CLIENTS = 5
MAX_HEADER_BYTES = 2048
MAX_BODY_BYTES = 1024
MAX_RFIDS = 40
RFID_CAPTURE_MS = 20000
HTTP_TIMEOUT_MS = 1800
HTTP_MAX_LIFETIME_MS = 5000


def due(now, deadline):
    return deadline is not None and time.ticks_diff(now, deadline) >= 0


def parse_request(raw):
    marker = raw.find(b"\r\n\r\n")
    if marker < 0:
        if len(raw) > MAX_HEADER_BYTES:
            raise ValueError("Header too large")
        return None
    if marker > MAX_HEADER_BYTES:
        raise ValueError("Header too large")
    lines = raw[:marker].decode().split("\r\n")
    method, path, version = lines[0].split()
    if version not in ("HTTP/1.0", "HTTP/1.1"):
        raise ValueError("Invalid HTTP version")
    headers = {}
    for line in lines[1:]:
        if ":" not in line:
            raise ValueError("Invalid header")
        key, value = line.split(":", 1)
        key = key.strip().lower()
        if not key or key in headers:
            raise ValueError("Duplicate or empty header")
        headers[key] = value.strip()
    if "transfer-encoding" in headers:
        raise ValueError("Transfer encoding unsupported")
    length_text = headers.get("content-length", "0")
    if not length_text or any(c not in "0123456789" for c in length_text):
        raise ValueError("Invalid content length")
    length = int(length_text)
    if length > MAX_BODY_BYTES:
        raise ValueError("Body too large")
    end = marker + 4 + length
    if len(raw) < end:
        return None
    if len(raw) != end:
        raise ValueError("Extra request bytes")
    return method, path, headers, raw[marker + 4:end]


def json_reply(status, value):
    body = json.dumps(value).encode()
    head = ("HTTP/1.1 " + status + "\r\nContent-Type: application/json\r\n"
            "Content-Length: " + str(len(body)) + "\r\nCache-Control: no-store\r\n"
            "Connection: close\r\n\r\n").encode()
    return head + body


class LCD1602:
    def __init__(self, i2c_bus, address):
        self.i2c = i2c_bus
        self.address = address
        self.backlight = 0x08

        time.sleep_ms(50)
        self._write4(0x30)
        time.sleep_ms(5)
        self._write4(0x30)
        time.sleep_ms(1)
        self._write4(0x30)
        self._write4(0x20)

        self.command(0x28)
        self.command(0x0C)
        self.command(0x06)
        self.clear()

    def _expander_write(self, value):
        self.i2c.writeto(self.address, bytes([value | self.backlight]))

    def _pulse(self, value):
        self._expander_write(value | 0x04)
        time.sleep_us(1)
        self._expander_write(value & ~0x04)
        time.sleep_us(50)

    def _write4(self, value):
        value &= 0xF0
        self._expander_write(value)
        self._pulse(value)

    def _send(self, value, mode):
        high = value & 0xF0
        low = (value << 4) & 0xF0

        self._expander_write(high | mode)
        self._pulse(high | mode)
        self._expander_write(low | mode)
        self._pulse(low | mode)

    def command(self, value):
        self._send(value, 0)

    def write_char(self, char):
        self._send(ord(char), 1)

    def clear(self):
        self.command(0x01)
        time.sleep_ms(2)

    def move_to(self, col, row):
        address = 0x00 if row == 0 else 0x40
        self.command(0x80 | (address + col))

    def putstr(self, text):
        for char in text:
            self.write_char(char)


class MFRC522:
    CommandReg = 0x01
    CommIEnReg = 0x02
    CommIrqReg = 0x04
    ErrorReg = 0x06
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    ModeReg = 0x11
    TxControlReg = 0x14
    TxASKReg = 0x15
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D

    PCD_IDLE = 0x00
    PCD_TRANSCEIVE = 0x0C
    PCD_SOFTRESET = 0x0F
    PICC_REQIDL = 0x26
    PICC_ANTICOLL = 0x93

    OK = 0
    ERROR = 2

    def __init__(self, sck, mosi, miso, rst, cs):
        self.spi = SPI(2, baudrate=1000000, polarity=0, phase=0,
                       sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
        self.rst = Pin(rst, Pin.OUT)
        self.cs = Pin(cs, Pin.OUT)

        self.cs.value(1)
        self.rst.value(0)
        time.sleep_ms(10)
        self.rst.value(1)
        time.sleep_ms(50)

        self.write_reg(self.CommandReg, self.PCD_SOFTRESET)
        time.sleep_ms(50)

        self.write_reg(self.TModeReg, 0x8D)
        self.write_reg(self.TPrescalerReg, 0x3E)
        self.write_reg(self.TReloadRegL, 30)
        self.write_reg(self.TReloadRegH, 0)
        self.write_reg(self.TxASKReg, 0x40)
        self.write_reg(self.ModeReg, 0x3D)
        self.antenna_on()

    def write_reg(self, reg, value):
        self.cs.value(0)
        self.spi.write(bytes([(reg << 1) & 0x7E, value & 0xFF]))
        self.cs.value(1)

    def read_reg(self, reg):
        self.cs.value(0)
        self.spi.write(bytes([((reg << 1) & 0x7E) | 0x80]))
        val = self.spi.read(1, 0x00)[0]
        self.cs.value(1)
        return val

    def set_bits(self, reg, mask):
        self.write_reg(reg, self.read_reg(reg) | mask)

    def antenna_on(self):
        temp = self.read_reg(self.TxControlReg)
        if (temp & 0x03) != 0x03:
            self.set_bits(self.TxControlReg, 0x03)

    def communicate(self, command, send_data):
        back_data = []
        back_len = 0
        status = self.ERROR

        irq_en = 0x00
        wait_irq = 0x00

        if command == self.PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_irq = 0x30

        self.write_reg(self.CommIEnReg, irq_en | 0x80)
        self.write_reg(self.CommIrqReg, 0x7F)
        self.set_bits(self.FIFOLevelReg, 0x80)
        self.write_reg(self.CommandReg, self.PCD_IDLE)

        for value in send_data:
            self.write_reg(self.FIFODataReg, value)

        self.write_reg(self.CommandReg, command)
        self.set_bits(self.BitFramingReg, 0x80)

        deadline = time.ticks_add(time.ticks_ms(), 8)
        i = 1
        while True:
            irq = self.read_reg(self.CommIrqReg)
            if irq & 0x01 or irq & wait_irq:
                break
            if time.ticks_diff(time.ticks_ms(), deadline) >= 0:
                i = 0
                break

        self.write_reg(self.BitFramingReg, self.read_reg(self.BitFramingReg) & 0x7F)

        if i != 0:
            error = self.read_reg(self.ErrorReg)
            if not (error & 0x1B) and not (irq & 0x01):
                status = self.OK
                n = self.read_reg(self.FIFOLevelReg)
                last_bits = self.read_reg(self.ControlReg) & 0x07
                if last_bits != 0:
                    back_len = (n - 1) * 8 + last_bits
                else:
                    back_len = n * 8

                if n == 0:
                    n = 1
                if n > 16:
                    n = 16

                for _ in range(n):
                    back_data.append(self.read_reg(self.FIFODataReg))

        return status, back_data, back_len

    def request(self):
        self.write_reg(self.BitFramingReg, 0x07)
        status, back_data, back_bits = self.communicate(self.PCD_TRANSCEIVE, [self.PICC_REQIDL])
        if status != self.OK or back_bits != 0x10:
            return self.ERROR
        return self.OK

    def anticoll(self):
        self.write_reg(self.BitFramingReg, 0x00)
        status, back_data, back_bits = self.communicate(self.PCD_TRANSCEIVE, [self.PICC_ANTICOLL, 0x20])
        if status != self.OK or len(back_data) != 5 or back_bits != 40:
            return None

        checksum = 0
        for i in range(4):
            checksum ^= back_data[i]
        if checksum != back_data[4]:
            return None

        if back_data[0] == 0x88:
            return None
        return back_data[:4]

    def read_uid(self):
        if self.request() != self.OK:
            return None
        uid = self.anticoll()
        if uid is None:
            return None

        result = ""
        for byte in uid:
            result += "{:02X}".format(byte)
        return result


class SmartDoor:
    def __init__(self):
        self.logs = {}
        self.buzzer = Pin(PIN_BUZZER, Pin.OUT)
        self.buzzer.value(0)
        self.beep_count = 0
        self.beep_on = False
        self.beep_deadline = None
        self.beep_ms = 60

        self.servo = PWM(Pin(PIN_SERVO), freq=50)
        self.servo.duty(0)
        self.servo_target = None
        self.servo_deadline = None
        self.open = False
        self.close_deadline = None
        self.move_servo(SERVO_LOCK_ANGLE)

        time.sleep_ms(SERVO_MOVE_MS)
        self.actuator_tick(time.ticks_ms())

        self.password = DEFAULT_PASSWORD
        self.rfids = DEFAULT_RFIDS[:]
        self.load_settings()
        self.entered = ""
        self.input_deadline = None
        self.message_deadline = None
        self.last_source = "BOOT"

        self.lcd = None
        self.lcd_target = [" " * 16, " " * 16]
        self.lcd_current = [None, None]
        self.lcd_retry = None
        self.init_lcd()

        self.rows = [Pin(p, Pin.IN) for p in ROW_PINS]
        self.cols = [Pin(p, Pin.IN, Pin.PULL_DOWN) for p in COL_PINS]
        self.key_candidate = None
        self.key_stable = None
        self.key_changed = time.ticks_ms()
        self.key_next = time.ticks_ms()
        self.button = Pin(PIN_BUTTON, Pin.IN)
        self.button_candidate = self.button.value()
        self.button_stable = 0
        self.button_changed = time.ticks_ms()

        self.rfid = None
        self.rfid_last_uid = None
        self.capture_deadline = None
        self.capture_uid = None
        self.capture_status = "idle"
        self.rfid_seen = time.ticks_ms()
        self.rfid_next = time.ticks_ms()
        self.rfid_retry = None
        self.init_rfid()

        self.wifi = network.WLAN(network.STA_IF)
        self.auto_net = None
        self.network_next = time.ticks_ms()
        self.network_retry = None
        self.init_network()

        self.server = None
        self.clients = []
        self.server_retry = None
        self.init_server()
        self.gc_next = time.ticks_add(time.ticks_ms(), 10000)
        self.ready()
        self.beep(80, 1)
        print("SMART DOOR READY; mat khau da nap; servo release:", SERVO_RELEASE)

    def log(self, tag, error):
        now = time.ticks_ms()
        previous = self.logs.get(tag)
        if previous is None or time.ticks_diff(now, previous) >= 5000:
            print("[" + tag + "]", error)
            self.logs[tag] = now

    def load_settings(self):
        try:
            nvs = NVS("SMARTDOOR")
        except OSError as exc:
            self.log("NVS", exc)
            return
        try:
            buf = bytearray(32)
            size = nvs.get_blob("password", buf)
            value = bytes(buf[:size]).decode()
            if value and len(value) <= 16 and all(c in "0123456789" for c in value):
                self.password = value
            else:
                self.log("PASSWORD", "Du lieu NVS khong hop le; dung mac dinh")
        except (OSError, ValueError, UnicodeError) as exc:
            self.log("PASSWORD", exc)
        try:
            buf = bytearray(2048)
            size = nvs.get_blob("rfids", buf)
            values = json.loads(bytes(buf[:size]).decode())
            if not isinstance(values, list):
                raise ValueError("RFID phai la danh sach")
            normalized = []
            for value in values:
                if not isinstance(value, str):
                    raise ValueError("UID khong hop le")
                value = value.upper()
                if len(value) != 8 or any(c not in "0123456789ABCDEF" for c in value):
                    raise ValueError("Chi ho tro UID 4 byte")
                if value not in normalized:
                    normalized.append(value)
            self.rfids = normalized
        except (OSError, ValueError, UnicodeError) as exc:
            self.log("RFID_NVS", exc)

    def move_servo(self, angle):
        angle = max(0, min(180, int(angle)))
        if angle == self.servo_target:
            return False
        pulse = 500 + 2000 * angle // 180
        self.servo.duty(pulse * 1023 // 20000)
        self.servo_target = angle
        self.servo_deadline = time.ticks_add(time.ticks_ms(), SERVO_MOVE_MS)
        return True

    def beep(self, duration=60, count=1):
        self.beep_ms = duration
        self.beep_count = count
        self.beep_on = count > 0
        self.buzzer.value(1 if self.beep_on else 0)
        self.beep_deadline = time.ticks_add(time.ticks_ms(), duration) if count > 0 else None

    def actuator_tick(self, now):
        if due(now, self.servo_deadline):
            if SERVO_RELEASE:
                self.servo.duty(0)
            self.servo_deadline = None
        if due(now, self.beep_deadline):
            if self.beep_on:
                self.buzzer.value(0)
                self.beep_on = False
                self.beep_count -= 1
                self.beep_deadline = time.ticks_add(now, 40) if self.beep_count > 0 else None
            else:
                self.buzzer.value(1)
                self.beep_on = True
                self.beep_deadline = time.ticks_add(now, self.beep_ms)

    def show(self, first="", second="", duration=None):
        lines = []
        for value in (first, second):
            value = str(value)[:16]
            lines.append(value + " " * (16 - len(value)))
        self.lcd_target = lines
        self.message_deadline = time.ticks_add(time.ticks_ms(), duration) if duration is not None else None

    def ready(self):
        if self.entered:
            self.show("NHAP MAT KHAU", "*" * len(self.entered))
        elif self.open:
            self.show("DOOR OPEN", "Tu khoa sau 5s")
        else:
            self.show("SMART DOOR", "Nhap mat khau")

    def init_lcd(self):
        try:
            bus = I2C(0, sda=Pin(LCD_SDA), scl=Pin(LCD_SCL), freq=100000)
            if LCD_ADDR not in bus.scan():
                raise OSError("Khong thay LCD 0x27")
            self.lcd = LCD1602(bus, LCD_ADDR)
            self.lcd_current = [None, None]
            self.lcd_retry = None
        except OSError as exc:
            self.lcd = None
            self.lcd_retry = time.ticks_add(time.ticks_ms(), 10000)
            self.log("LCD", exc)

    def lcd_tick(self):
        if self.lcd is None:
            return
        try:
            for row in range(2):
                target = self.lcd_target[row]
                current = self.lcd_current[row]
                if current == target:
                    continue
                if current is None:
                    current = "\x00" * 16
                start = 0
                while start < 16 and current[start] == target[start]:
                    start += 1
                end = min(start + 4, 16)
                self.lcd.move_to(start, row)
                self.lcd.putstr(target[start:end])
                self.lcd_current[row] = current[:start] + target[start:end] + current[end:]
                break
        except OSError as exc:
            self.lcd = None
            self.lcd_retry = time.ticks_add(time.ticks_ms(), 10000)
            self.log("LCD", exc)

    def grant(self, source):
        was_open = self.open
        self.move_servo(SERVO_OPEN_ANGLE)
        self.open = True
        self.close_deadline = time.ticks_add(time.ticks_ms(), DOOR_OPEN_MS)
        self.entered = ""
        self.input_deadline = None
        self.last_source = source
        self.show("ACCESS GRANTED", source)
        if not was_open:
            self.beep(80, 2)
        print("MO/GIA HAN CUA:", source)

    def lock(self, source):
        self.close_deadline = None
        self.entered = ""
        self.input_deadline = None
        changed = self.move_servo(SERVO_LOCK_ANGLE)
        self.open = False
        self.last_source = source
        if changed:
            self.show("DOOR LOCKED", source, 1000)
            print("KHOA CUA:", source)
        return changed

    def deny(self):
        self.entered = ""
        self.input_deadline = None
        self.show("ACCESS DENIED", "Sai thong tin", 1200)
        self.beep(120, 2)

    def key_scan(self, now):
        found = None
        count = 0
        for r in range(4):
            self.rows[r].init(Pin.OUT, value=1)
            try:
                time.sleep_us(5)
                for c in range(4):
                    if self.cols[c].value():
                        count += 1
                        found = KEYPAD_MAP[r][c]
            finally:
                self.rows[r].init(Pin.IN)
        raw = found if count == 1 else None
        if raw != self.key_candidate:
            self.key_candidate = raw
            self.key_changed = now
        elif raw != self.key_stable and time.ticks_diff(now, self.key_changed) >= 25:
            self.key_stable = raw
            if raw is not None:
                self.process_key(raw)

    def process_key(self, key):
        if key == "D":
            self.lock("KEY D")
            self.beep(35, 1)
            return
        if key not in "0123456789*#":
            return
        self.beep(35, 1)
        if key == "*":
            self.entered = ""
            self.input_deadline = None
            self.show("NHAP MAT KHAU", "")
        elif key == "#":
            if self.entered == self.password:
                self.grant("PASSWORD")
            else:
                self.deny()
        else:
            if len(self.entered) < len(self.password):
                self.entered += key
            self.input_deadline = time.ticks_add(time.ticks_ms(), PASSWORD_TIMEOUT_MS)
            self.show("NHAP MAT KHAU", "*" * len(self.entered))
            if len(self.entered) == len(self.password):
                if self.entered == self.password:
                    self.grant("PASSWORD")
                else:
                    self.deny()

    def button_tick(self, now):
        raw = self.button.value()
        if raw != self.button_candidate:
            self.button_candidate = raw
            self.button_changed = now
        elif raw != self.button_stable and time.ticks_diff(now, self.button_changed) >= 30:
            self.button_stable = raw
            if raw:
                self.grant("BUTTON")

    def init_rfid(self):
        try:
            self.rfid = MFRC522(SCK_PIN, MOSI_PIN, MISO_PIN, RST_PIN, SS_PIN)
            version = self.rfid.read_reg(0x37)
            if version in (0x00, 0xFF):
                raise OSError("RC522 khong phan hoi; kiem tra day/nguon 3.3V")
            self.rfid_retry = None
        except OSError as exc:
            self.rfid = None
            self.rfid_retry = time.ticks_add(time.ticks_ms(), 10000)
            self.log("RFID", exc)

    def rfid_tick(self, now):
        if self.rfid is None or not due(now, self.rfid_next):
            return
        self.rfid_next = time.ticks_add(now, RFID_SCAN_MS)
        try:
            uid = self.rfid.read_uid()
        except OSError as exc:
            self.rfid = None
            self.rfid_retry = time.ticks_add(time.ticks_ms(), 10000)
            self.log("RFID", exc)
            return
        if uid is None:
            if time.ticks_diff(now, self.rfid_seen) >= 600:
                self.rfid_last_uid = None
            return
        self.rfid_seen = now
        if self.capture_deadline is not None and not due(now, self.capture_deadline):
            self.capture_uid = uid
            self.capture_deadline = None
            self.capture_status = "captured"
            self.rfid_last_uid = uid
            self.show("RFID SCANNED", "Them the tren web", 1500)
            self.beep(60, 2)
            return
        if uid == self.rfid_last_uid:
            return
        self.rfid_last_uid = uid
        if uid in self.rfids:
            self.grant("RFID " + uid)
        else:
            self.show("RFID TU CHOI", uid, 1200)
            self.beep(120, 2)

    def init_network(self):
        try:
            self.auto_net = AutoNetwork(2, WIFI_SSID, WIFI_PASSWORD, CONTROL_KEY)
            self.wifi = self.auto_net.sta
            self.network_retry = None
        except OSError as exc:
            self.auto_net = None
            self.network_retry = time.ticks_add(time.ticks_ms(), 10000)
            self.log("WIFI", exc)

    def network_tick(self, now):
        if self.auto_net is not None and due(now, self.network_next):
            self.network_next = time.ticks_add(now, 100)
            try:
                self.auto_net.tick()
            except OSError as exc:
                self.network_next = time.ticks_add(time.ticks_ms(), 2000)
                self.log("WIFI", exc)

    def init_server(self):
        server = None
        try:
            server = socket.socket()
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", 80))
            server.listen(MAX_CLIENTS)
            server.setblocking(False)
            self.server = server
            self.server_retry = None
        except OSError as exc:
            if server is not None:
                server.close()
            self.server = None
            self.server_retry = time.ticks_add(time.ticks_ms(), 5000)
            self.log("HTTP", exc)

    def api_state(self):
        try:
            ip = self.wifi.ifconfig()[0]
        except OSError:
            ip = "0.0.0.0"
        left = 0
        if self.open and self.close_deadline is not None:
            left = max(0, time.ticks_diff(self.close_deadline, time.ticks_ms()))
        return {"open": self.open, "ip": ip, "source": "servo_command",
                "auto_close_ms": left, "servo_moving": self.servo_deadline is not None,
                "door_admin": True, "firmware": "NOVA_DOOR_FAST_2"}

    def normalize_uid(self, value):
        if not isinstance(value, str):
            raise ValueError("Invalid UID")
        value = value.upper().replace(":", "").replace("-", "").replace(" ", "")
        if len(value) != 8 or any(c not in "0123456789ABCDEF" for c in value):
            raise ValueError("UID must contain 8 hexadecimal digits")
        return value

    def admin_state(self):
        if due(time.ticks_ms(), self.capture_deadline):
            self.capture_deadline = None
            self.capture_status = "timeout"
        return {"ok": True, "rfids": self.rfids[:], "max_rfids": MAX_RFIDS,
                "scan_status": self.capture_status, "captured_uid": self.capture_uid}

    def admin_command(self, action, data):
        if not isinstance(data, dict):
            raise ValueError("JSON object required")


        if action == "password":
            if not isinstance(data.get("current_password"), str) or data.get("current_password") != self.password:
                return "401 Unauthorized", {"error": "Wrong current password"}

            new = data.get("new_password")
            if not isinstance(new, str) or not 2 <= len(new) <= 16 or any(c not in "0123456789" for c in new):
                raise ValueError("Password must contain 2 to 16 digits")
            if new != self.password:
                nvs = NVS("SMARTDOOR")
                nvs.set_blob("password", new.encode())
                nvs.commit()
                self.password = new
            self.entered = ""
            self.input_deadline = None
            self.capture_deadline = None
            self.capture_uid = None
            self.capture_status = "idle"
            self.show("PASSWORD SAVED", "Doi tren web", 1200)
            return "200 OK", {"ok": True, "password_changed": True}


        if action == "rfid_scan":
            if self.rfid is None:
                return "503 Service Unavailable", {"error": "RFID reader unavailable"}
            self.capture_uid = None
            self.capture_status = "scanning"
            self.capture_deadline = time.ticks_add(time.ticks_ms(), RFID_CAPTURE_MS)
            self.show("QUET THE MOI", "Dang cho 20 giay", RFID_CAPTURE_MS)
            return "200 OK", self.admin_state()

        if action in ("rfid_list", "rfid_status"):
            return "200 OK", self.admin_state()

        values = self.rfids[:]
        if action == "rfid_add":
            uid = self.normalize_uid(data.get("uid"))
            if uid not in values:
                if len(values) >= MAX_RFIDS:
                    raise ValueError("RFID list full")
                values.append(uid)
        elif action == "rfid_remove":
            uid = self.normalize_uid(data.get("uid"))
            if uid not in values:
                raise ValueError("Card not found")
            values.remove(uid)
        elif action == "rfid_replace":
            old_uid = self.normalize_uid(data.get("old_uid"))
            new_uid = self.normalize_uid(data.get("uid"))
            if old_uid not in values:
                raise ValueError("Card not found")
            if new_uid != old_uid and new_uid in values:
                raise ValueError("Card already exists")
            values[values.index(old_uid)] = new_uid
        else:
            raise ValueError("Unknown command")

        if values != self.rfids:
            nvs = NVS("SMARTDOOR")
            nvs.set_blob("rfids", json.dumps(values).encode())
            nvs.commit()
            self.rfids = values

        self.capture_deadline = None
        self.capture_uid = None
        self.capture_status = "idle"
        self.show("RFID SAVED", "Da luu danh sach", 1200)
        return "200 OK", self.admin_state()

    def response(self, raw):
        try:
            request = parse_request(raw)
            if request is None:
                raise ValueError("Incomplete request")
            method, path, headers, body = request
            if method == "GET" and path == "/api":
                return json_reply("200 OK", self.api_state())
            actions = ("open", "lock", "password", "rfid_list", "rfid_scan",
                       "rfid_status", "rfid_add", "rfid_remove", "rfid_replace")
            if method != "POST" or not path.startswith("/cmd/") or path[5:] not in actions:
                return json_reply("404 Not Found", {"error": "Unknown command"})
            if not CONTROL_KEY or headers.get("x-control-key") != CONTROL_KEY:
                return json_reply("403 Forbidden", {"error": "Control key rejected"})
            action = path[5:]
            if action == "open":
                self.grant("WEB")
                return json_reply("200 OK", self.api_state())
            if action == "lock":
                self.lock("WEB")
                return json_reply("200 OK", self.api_state())
            data = json.loads(body.decode()) if body else {}
            status, result = self.admin_command(action, data)
            return json_reply(status, result)
        except (ValueError, UnicodeError) as exc:
            return json_reply("400 Bad Request", {"error": str(exc)})
        except OSError:

            return json_reply("503 Service Unavailable", {"error": "Settings write failed"})

    def close_client(self, item):
        try:
            item[0].close()
        except OSError:
            pass
        self.clients.remove(item)

    def web_tick(self):
        if self.server is None:
            return

        for _ in range(2):
            try:
                client, address = self.server.accept()
                if len(self.clients) >= MAX_CLIENTS:
                    client.close()
                    continue
                try:
                    client.setblocking(False)
                except OSError:
                    client.close()
                    raise
                now = time.ticks_ms()

                self.clients.append([client, b"", None, now, now])
            except OSError as exc:
                if not would_block(exc):
                    self.log("HTTP_ACCEPT", exc)
                break
        for item in self.clients[:]:
            client = item[0]
            now = time.ticks_ms()
            done = (
                time.ticks_diff(now, item[4]) >= HTTP_TIMEOUT_MS
                or time.ticks_diff(now, item[3]) >= HTTP_MAX_LIFETIME_MS
            )
            if done:
                self.close_client(item)
                continue
            if item[2] is None:
                try:
                    chunk = client.recv(512)
                    if chunk is None:
                        continue
                    if not chunk:
                        self.close_client(item)
                        continue
                    item[1] += chunk
                    item[4] = time.ticks_ms()
                    try:
                        complete = parse_request(item[1])
                        if complete is None:
                            continue
                        item[2] = memoryview(self.response(item[1]))
                    except (ValueError, UnicodeError):
                        item[2] = memoryview(json_reply(
                            "400 Bad Request", {"error": "Invalid HTTP request"}
                        ))
                    item[1] = b""
                except OSError as exc:
                    if not would_block(exc):
                        self.close_client(item)
                    continue

            if item[2] is not None:
                try:
                    sent = client.send(item[2])
                    if sent is None:
                        continue
                    if sent <= 0:
                        self.close_client(item)
                        continue
                    item[2] = item[2][sent:]
                    item[4] = time.ticks_ms()
                    if not len(item[2]):
                        self.close_client(item)
                except OSError as exc:
                    if not would_block(exc):
                        self.close_client(item)

    def timers_tick(self, now):
        self.actuator_tick(now)
        if due(now, self.capture_deadline):
            self.capture_deadline = None
            self.capture_status = "timeout"
        if self.open and due(now, self.close_deadline):
            self.lock("AUTO 5S")
        if due(now, self.input_deadline):
            self.entered = ""
            self.input_deadline = None
            self.ready()
        if due(now, self.message_deadline):
            self.message_deadline = None
            self.ready()

    def maintenance_tick(self, now):

        if self.servo_deadline is not None or self.clients:
            return
        if self.server is None and due(now, self.server_retry):
            self.init_server()
        elif self.lcd is None and due(now, self.lcd_retry):
            self.init_lcd()
        elif self.rfid is None and due(now, self.rfid_retry):
            self.init_rfid()
        elif self.auto_net is None and due(now, self.network_retry):
            self.init_network()
        elif due(now, self.gc_next):
            gc.collect()
            self.gc_next = time.ticks_add(time.ticks_ms(), 10000)

    def step(self):
        now = time.ticks_ms()
        self.actuator_tick(now)
        self.button_tick(now)
        if due(now, self.key_next):
            self.key_next = time.ticks_add(now, 5)
            self.key_scan(now)
        self.network_tick(time.ticks_ms())
        self.web_tick()
        self.rfid_tick(time.ticks_ms())
        self.timers_tick(time.ticks_ms())
        self.lcd_tick()
        self.web_tick()
        self.network_tick(time.ticks_ms())
        self.timers_tick(time.ticks_ms())
        self.maintenance_tick(time.ticks_ms())

    def cleanup(self):
        self.buzzer.value(0)
        self.servo.duty(0)
        for item in self.clients[:]:
            self.close_client(item)
        if self.server is not None:
            self.server.close()


def would_block(exc):
    return bool(exc.args) and exc.args[0] in (11, 35, 10035)


def main():
    door = SmartDoor()
    try:
        while True:
            door.step()
            time.sleep_ms(2)
    finally:
        door.cleanup()


if __name__ == "__main__":
    main()
