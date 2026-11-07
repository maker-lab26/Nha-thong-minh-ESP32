print("NOVA ALL FAST - ESP32 SO 1")
from machine import Pin, SoftI2C, PWM, ADC
from array import array
import time
import framebuf
import micropython
import network
import socket
import json
import os
import dht
from config import WIFI_SSID, WIFI_PASSWORD, CONTROL_KEY
from auto_network import AutoNetwork
auto_net = None
web_ready = False


WIFI_NAME = WIFI_SSID


RELAY_ACTIVE_LOW = True
ON = 0 if RELAY_ACTIVE_LOW else 1
OFF = 1 - ON

AUTO_DIFF = 3
REPEAT_MS = 180

relay = Pin(26, Pin.OUT, value=OFF)
sensor = dht.DHT11(Pin(25))


SEPARATE_RELAY_ACTIVE_LOW = False
Q_ON = 0 if SEPARATE_RELAY_ACTIVE_LOW else 1
Q_OFF = 1 - Q_ON
separate_relay = Pin(27, Pin.OUT, value=Q_OFF)
separate_on = False


GAS_ON_ADC = 2400
GAS_OFF_ADC = 1800
GAS_CLEAR_MS = 3000


GAS_SETTINGS_FILE = 'gas_settings.json'

def validate_gas_thresholds(data):
    if not isinstance(data, dict):
        raise ValueError('Expected JSON object')
    on, off = data.get('on'), data.get('off')
    if type(on) is not int or type(off) is not int or not 0 <= off < on <= 4095:
        raise ValueError('Require 0 <= off < on <= 4095')
    return on, off

try:
    with open(GAS_SETTINGS_FILE) as settings:
        GAS_ON_ADC, GAS_OFF_ADC = validate_gas_thresholds(json.load(settings))
except (OSError, ValueError, TypeError):
    print('Gas: dung nguong mac dinh 2400 / 1800')

def save_gas_thresholds(data):
    global GAS_ON_ADC, GAS_OFF_ADC, gas_clear_since, display_dirty
    on, off = validate_gas_thresholds(data)
    if (on, off) != (GAS_ON_ADC, GAS_OFF_ADC):
        with open(GAS_SETTINGS_FILE + '.tmp', 'w') as settings:
            settings.write(json.dumps({'on': on, 'off': off}))
        os.rename(GAS_SETTINGS_FILE + '.tmp', GAS_SETTINGS_FILE)
        GAS_ON_ADC, GAS_OFF_ADC = on, off
        gas_clear_since = None
        display_dirty = True
    update_gas()


GAS_OUTPUT_ACTIVE_LOW = False


BUZZER_PASSIVE = False
BUZZER_ACTIVE_LOW = False

G_ON = 0 if GAS_OUTPUT_ACTIVE_LOW else 1
G_OFF = 1 - G_ON
B_ON = 0 if BUZZER_ACTIVE_LOW else 1
B_OFF = 1 - B_ON


gas_sensor = ADC(Pin(32))
gas_sensor.atten(ADC.ATTN_11DB)
gas_sensor.width(ADC.WIDTH_12BIT)

gas_fan = Pin(18, Pin.OUT, value=G_OFF)

if BUZZER_PASSIVE:
    buzzer = PWM(Pin(33), freq=2000, duty=0)
else:
    buzzer = Pin(33, Pin.OUT, value=B_OFF)

gas_alarm = False
gas_muted = False
gas_valid = False
gas_clear_since = None
buzzer_state = None
gas_raw = None
last_gas_error = None
last_network_tick = time.ticks_ms()
last_network_error = None
last_gas = time.ticks_ms()
last_gas_check = time.ticks_add(time.ticks_ms(), -100)
last_gas_print = time.ticks_add(time.ticks_ms(), -1000)

target = 26
temperature = None
humidity = None
last_good = None

fan_on = False
ac_on = False
auto_target = False
display_dirty = True
remote_ready = False


def set_buzzer(on):
    global buzzer_state
    on = bool(on)
    if buzzer_state == on:
        return
    if BUZZER_PASSIVE:
        buzzer.duty(512 if on else 0)
    else:
        buzzer.value(B_ON if on else B_OFF)
    buzzer_state = on


def gas_command(action):
    global gas_muted, display_dirty
    if action == 'gas_mute':

        gas_muted = bool(gas_alarm)
    elif action == 'gas_unmute':
        gas_muted = False
    else:
        raise ValueError('Unknown gas command')
    set_buzzer(gas_alarm and not gas_muted)
    display_dirty = True


def update_gas():
    global gas_alarm, gas_raw, last_gas, last_gas_check
    global last_gas_print, display_dirty, gas_muted, gas_valid, last_gas_error, gas_clear_since

    now = time.ticks_ms()

    if time.ticks_diff(now, last_gas_check) < 100:
        return

    last_gas_check = now


    try:
        gas_raw = sum(gas_sensor.read() for _ in range(8)) // 8
        gas_valid = True
    except OSError:
        gas_valid = False
        gas_clear_since = None

        if last_gas_error is None or time.ticks_diff(now, last_gas_error) >= 5000:
            print('Gas ADC: chua doc duoc cam bien')
            last_gas_error = now
        return
    previous = gas_alarm

    if gas_raw >= GAS_ON_ADC:
        last_gas = now
        gas_clear_since = None
        gas_alarm = True
    elif gas_alarm:
        if gas_raw > GAS_OFF_ADC:
            last_gas = now
            gas_clear_since = None
        elif gas_clear_since is None:
            gas_clear_since = now
        elif time.ticks_diff(now, gas_clear_since) >= GAS_CLEAR_MS:
            gas_alarm = False
            gas_clear_since = None

    if gas_alarm != previous:
        gas_muted = False
    desired_fan = G_ON if gas_alarm else G_OFF
    if gas_fan.value() != desired_fan:
        gas_fan.value(desired_fan)
    set_buzzer(gas_alarm and not gas_muted)

    if gas_alarm != previous:
        display_dirty = True
        print(
            'BAO GAS: BAT QUAT HUT + COI' if gas_alarm
            else 'HET BAO GAS: TAT QUAT HUT + COI'
        )

    if time.ticks_diff(now, last_gas_print) >= 1000:
        last_gas_print = now
        display_dirty = True


def service_io():
    global last_network_tick, last_network_error
    update_gas()
    process_remote()
    if web_ready:
        handle_web()
    now = time.ticks_ms()
    if auto_net is not None and time.ticks_diff(now, last_network_tick) >= 100:
        last_network_tick = now
        try:
            auto_net.tick()
        except OSError:
            if last_network_error is None or time.ticks_diff(now, last_network_error) >= 5000:
                print('WiFi: dang cho ket noi lai')
                last_network_error = now


def separate_command(action):
    global separate_on, display_dirty

    if action == 'q_toggle':
        separate_on = not separate_on
    elif action in ('q_on', 'q_off'):
        separate_on = action == 'q_on'
    else:
        return

    separate_relay.value(Q_ON if separate_on else Q_OFF)
    print('Quat rieng:', 'BAT' if separate_on else 'TAT')
    display_dirty = True


def sensor_valid():
    return (
        last_good is not None
        and time.ticks_diff(time.ticks_ms(), last_good) < 10000
    )

def set_fan(value):
    global fan_on, display_dirty

    value = bool(value)

    if value != fan_on:
        display_dirty = True

    fan_on = value
    relay.value(ON if fan_on else OFF)

def choose_target():
    global target, display_dirty

    if sensor_valid():
        new_target = temperature - AUTO_DIFF

        if new_target != target:
            target = new_target
            display_dirty = True

def thermostat():
    if not ac_on:
        return

    if not sensor_valid():
        set_fan(False)
    elif temperature >= target + 1:
        set_fan(True)
    elif temperature <= target - 1:
        set_fan(False)

def command(action):
    global target, ac_on, auto_target, display_dirty

    if action in ('gas_mute', 'gas_unmute'):
        gas_command(action)
        return

    if action.startswith('q_'):
        separate_command(action)
        return

    if action in ('toggle', 'fan_on', 'fan_off'):
        ac_on = False
        set_fan(
            not fan_on if action == 'toggle'
            else action == 'fan_on'
        )

    elif action in ('up', 'down'):
        target = min(
            32, max(20, target + (1 if action == 'up' else -1))
        )
        auto_target = False

    elif action in ('ac_on', 'ac_off'):
        ac_on = action == 'ac_on'
        set_fan(False)

        if ac_on and auto_target:
            choose_target()

    elif action == 'auto':
        ac_on = auto_target = True
        choose_target()

    elif action == 'manual':
        auto_target = False

    thermostat()
    display_dirty = True


micropython.alloc_emergency_exception_buf(100)

ir = Pin(14, Pin.IN, Pin.PULL_UP)
ir_queue = array('H', [0] * 32)

queue_read = 0
queue_write = 0
last_edge = time.ticks_us()
bit_index = -1

ir_address = 0
ir_button = 0
ir_inverse = 0

last_ir_button = -1
last_ir_time = time.ticks_ms()

def ir_irq(pin):
    global last_edge, bit_index
    global ir_address, ir_button, ir_inverse
    global queue_write, last_ir_button, last_ir_time

    now = time.ticks_us()
    duration = time.ticks_diff(now, last_edge)
    last_edge = now

    if duration > 20000 or duration <= 0:
        bit_index = -1
        return

    if bit_index == -1:
        if 12000 < duration < 15000:
            bit_index = 0
            ir_address = 0
            ir_button = 0
            ir_inverse = 0

        elif 10000 < duration < 12500:
            now_ms = time.ticks_ms()

            if last_ir_button >= 0:
                if time.ticks_diff(now_ms, last_ir_time) < 300:
                    last_ir_time = now_ms
                    next_write = (queue_write + 1) & 31

                    if next_write != queue_read:
                        ir_queue[queue_write] = last_ir_button | 256
                        queue_write = next_write
        return

    if 750 < duration < 1550:
        value = 0
    elif 1700 < duration < 2850:
        value = 1
    else:
        bit_index = -1
        return

    if value:
        if bit_index < 16:
            ir_address |= 1 << bit_index
        elif bit_index < 24:
            ir_button |= 1 << (bit_index - 16)
        else:
            ir_inverse |= 1 << (bit_index - 24)

    bit_index += 1

    if bit_index == 32:
        bit_index = -1

        if ir_address == 0xFF00:
            if (ir_button ^ ir_inverse) == 255:
                last_ir_button = ir_button
                last_ir_time = time.ticks_ms()
                next_write = (queue_write + 1) & 31

                if next_write != queue_read:
                    ir_queue[queue_write] = ir_button
                    queue_write = next_write

last_adjust = time.ticks_add(time.ticks_ms(), -REPEAT_MS)

def process_remote():
    global queue_read, last_adjust

    while queue_read != queue_write:
        event = ir_queue[queue_read]
        queue_read = (queue_read + 1) & 31

        code = event & 255
        repeat = bool(event & 256)
        now = time.ticks_ms()

        if not repeat:
            print('Remote: 0x%02X' % code)

        if code in (0x47, 0x43):
            if not repeat:
                command(
                    'toggle' if code == 0x47 else 'q_toggle'
                )

        elif code in (0x15, 0x07):
            if repeat and time.ticks_diff(now, last_adjust) < REPEAT_MS:
                continue

            last_adjust = now
            command('up' if code == 0x15 else 'down')

def remote_busy():
    return time.ticks_diff(time.ticks_us(), last_edge) < 150000


class OLED(framebuf.FrameBuffer):
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        self.buffer = bytearray(1024)

        self.packet = bytearray(129)
        self.packet[0] = 0x40

        super().__init__(
            self.buffer, 128, 64, framebuf.MONO_VLSB
        )

        commands = (
            0xAE, 0x20, 0x00, 0x40, 0xA1, 0xC8,
            0xA8, 0x3F, 0xD3, 0x00, 0xDA, 0x12,
            0xD5, 0x80, 0xD9, 0xF1, 0xDB, 0x30,
            0x81, 0xCF, 0x8D, 0x14, 0xA4, 0xA6,
            0x2E, 0xAF
        )

        for c in commands:
            self.command(c)

        self.fill(0)
        self.text('OLED OK', 32, 25)
        self.show()

    def command(self, c):
        self.bus.writeto(self.address, bytes((0x80, c)))

    def show(self):
        for c in (0x21, 0, 127, 0x22, 0, 7):
            self.command(c)

        for offset in range(0, 1024, 128):
            self.packet[1:] = self.buffer[offset:offset + 128]
            self.bus.writeto(self.address, self.packet)

            if remote_ready:
                service_io()

oled = None
last_oled_try = time.ticks_ms()
font = framebuf.FrameBuffer(
    bytearray(16), 16, 8, framebuf.MONO_VLSB
)

def connect_oled():
    global oled, last_oled_try, display_dirty

    oled = None
    last_oled_try = time.ticks_ms()

    try:
        sda = Pin(21, Pin.IN, Pin.PULL_UP)
        scl = Pin(22, Pin.IN, Pin.PULL_UP)
        time.sleep_ms(10)

        if not sda.value() or not scl.value():
            print('OLED: SDA/SCL bi keo LOW. Kiem tra day.')
            return

        bus = SoftI2C(sda=Pin(21), scl=Pin(22), freq=100000)
        devices = bus.scan()
        print('I2C:', [hex(x) for x in devices])

        if len(devices) > 16:
            print('OLED: Bus I2C loi.')
            return

        for address in (0x3C, 0x3D):
            if address in devices:
                oled = OLED(bus, address)
                display_dirty = True
                print('OLED OK')
                return

        print('OLED: Khong tim thay man.')

    except OSError as e:
        oled = None
        print('OLED:', e)

def update_oled():
    global oled, display_dirty, last_oled_try

    if oled is None:
        if time.ticks_diff(time.ticks_ms(), last_oled_try) >= 5000:
            connect_oled()
        return

    if not display_dirty:
        return

    display_dirty = False

    try:
        oled.fill(0)

        if ac_on and auto_target:
            title = 'AUTO T-3C'
        elif ac_on:
            title = 'AUTO AC'
        else:
            title = 'MANUAL FAN'

        oled.text(
            ('GAS! COI IM' if gas_muted else 'GAS! HUT+COI ON') if gas_alarm else title,
            0, 0
        )

        font.fill(0)
        font.text(str(target), 0, 0)

        for y in range(8):
            for x in range(16):
                if font.pixel(x, y):
                    oled.fill_rect(
                        2 + x * 3, 14 + y * 3, 3, 3, 1
                    )

        oled.text('C', 53, 28)

        if sensor_valid():
            oled.text('T:%sC' % temperature, 65, 15)
            oled.text('H:%s%%' % humidity, 65, 28)
        else:
            oled.text('DHT ERR', 65, 15)

        if (time.ticks_ms() // 2000) % 2:
            oled.text('GAS ADC:%s' % gas_raw, 0, 43)
        else:
            oled.text(
                'AC:%s Q:%s' % (
                    'ON' if fan_on else 'OFF',
                    'ON' if separate_on else 'OFF'
                ),
                0, 43
            )

        ip = sta.ifconfig()[0] if sta.isconnected() else 'WiFi OFF'
        oled.text(ip, 0, 55)
        oled.show()

    except OSError as e:
        print('OLED:', e)
        oled = None
        display_dirty = True
        last_oled_try = time.ticks_ms()

update_gas()
connect_oled()


auto_net = AutoNetwork(1, WIFI_SSID, WIFI_PASSWORD, CONTROL_KEY)
sta = auto_net.sta


def read_sensor():
    global temperature, humidity, last_good, display_dirty

    try:
        sensor.measure()
        t = sensor.temperature()
        h = sensor.humidity()

        if 0 <= t <= 60 and 0 <= h <= 100:
            temperature = t
            humidity = h
            last_good = time.ticks_ms()

            if ac_on and auto_target:
                choose_target()

    except OSError:
        pass

    display_dirty = True




def state_json():
    valid = sensor_valid()

    return json.dumps({
        'firmware': 'NOVA_ALL_FAST_1',
        'target': target,
        'temp': temperature if valid else None,
        'hum': humidity if valid else None,
        'valid': valid,
        'fan': fan_on,
        'ac': ac_on,
        'auto': auto_target,
        'q_on': separate_on,
        'gas': gas_alarm,
        'gas_valid': gas_valid,
        'gas_muted': gas_muted,
        'buzzer_on': bool(buzzer_state),
        'gas_fan': gas_alarm,
        'gas_mute_control': True,
        'gas_threshold_control': True,
        'gas_raw': gas_raw,
        'gas_percent': (
            round(gas_raw * 100 / 4095, 1)
            if gas_raw is not None else None
        ),
        'gas_threshold': GAS_ON_ADC,
        'gas_off_threshold': GAS_OFF_ADC,
        'ip': sta.ifconfig()[0] if sta.isconnected() else None
    })

MAX_HEADER_BYTES = 2048
MAX_BODY_BYTES = 1024
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


def reply_bytes(body, content_type, status='200 OK'):
    if isinstance(body, str):
        body = body.encode('utf-8')
    header = ('HTTP/1.1 ' + status + '\r\nContent-Type: ' + content_type
              + '\r\nContent-Length: ' + str(len(body))
              + '\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n').encode()
    return header + body


server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', 80))
server.listen(5)
server.setblocking(False)
web_clients = []
ACTIONS = ('toggle', 'up', 'down', 'fan_on', 'fan_off',
           'ac_on', 'ac_off', 'auto', 'manual',
           'q_toggle', 'q_on', 'q_off', 'gas_mute', 'gas_unmute')


def web_response(raw):
    try:
        parsed = parse_request(raw)
        if parsed is None:
            raise ValueError('Incomplete request')
        method, path, headers, body = parsed
        if method == 'GET' and path == '/api':
            return reply_bytes(state_json(), 'application/json')
        if method == 'POST' and path == '/cmd/gas_threshold':
            try:
                save_gas_thresholds(json.loads(body.decode()))
            except (ValueError, TypeError):
                return reply_bytes('{"error":"Require 0 <= off < on <= 4095"}', 'application/json', '400 Bad Request')
            except OSError:
                return reply_bytes('{"error":"Cannot save gas settings"}', 'application/json', '500 Internal Server Error')
            return reply_bytes(state_json(), 'application/json')
        if method == 'POST' and path.startswith('/cmd/') and path[5:] in ACTIONS:
            command(path[5:])
            return reply_bytes(state_json(), 'application/json')
        return reply_bytes('{"error":"Unknown command"}', 'application/json', '404 Not Found')
    except (ValueError, UnicodeError):
        return reply_bytes('{"error":"Bad request"}', 'application/json', '400 Bad Request')


def close_web_client(item):
    try:
        item[0].close()
    except OSError:
        pass
    web_clients.remove(item)


def socket_waiting(error):
    return bool(error.args) and error.args[0] in (11, 35, 10035)


def handle_web():
    for _ in range(2):
        try:
            client, address = server.accept()
            if len(web_clients) >= 5:
                client.close()
                continue
            try:
                client.setblocking(False)
            except OSError:
                client.close()
                raise
            now = time.ticks_ms()
            web_clients.append([client, b'', None, now, now])
        except OSError as error:
            break
    for item in web_clients[:]:
        client = item[0]
        now = time.ticks_ms()
        if (time.ticks_diff(now, item[4]) >= 1800
                or time.ticks_diff(now, item[3]) >= 5000):
            close_web_client(item)
            continue
        if item[2] is None:
            try:
                chunk = client.recv(512)
                if chunk is None:
                    continue
                if not chunk:
                    close_web_client(item)
                    continue
                item[1] += chunk
                item[4] = time.ticks_ms()
                try:
                    if parse_request(item[1]) is None:
                        continue
                    item[2] = memoryview(web_response(item[1]))
                except (ValueError, UnicodeError):
                    item[2] = memoryview(reply_bytes(
                        '{"error":"Bad request"}', 'application/json', '400 Bad Request'
                    ))
                item[1] = b''
            except OSError as error:
                if not socket_waiting(error):
                    close_web_client(item)
                continue
        try:
            sent = client.send(item[2][:1024])
            if sent is None:
                continue
            if sent <= 0:
                close_web_client(item)
                continue
            item[2] = item[2][sent:]
            item[4] = time.ticks_ms()
            if not len(item[2]):
                close_web_client(item)
        except OSError as error:
            if not socket_waiting(error):
                close_web_client(item)


try:
    ir.irq(handler=ir_irq, trigger=Pin.IRQ_FALLING, hard=True)
    print('Remote: ngat cung')
except TypeError:
    ir.irq(handler=ir_irq, trigger=Pin.IRQ_FALLING)
    print('Remote: ngat mem')


web_ready = True
remote_ready = True
last_sensor = time.ticks_add(time.ticks_ms(), -2500)
last_draw = time.ticks_add(time.ticks_ms(), -120)
last_wifi = time.ticks_ms()
printed_ip = None

print('SAN SANG!')
print('Dieu hoa: GPIO26, remote 0x47 BAT/TAT')
print('Nhiet do: 0x15 TANG, 0x07 GIAM')
print('Quat rieng: GPIO27, remote 0x43 BAT/TAT')
print('Gas: AO GPIO32, S8050 quat GPIO18, S8050 coi GPIO33')

try:
    while True:
        service_io()
        now = time.ticks_ms()

        if not remote_busy() and time.ticks_diff(now, last_sensor) >= 2500:
            last_sensor = now
            read_sensor()
            service_io()

        thermostat()

        if time.ticks_diff(now, last_draw) >= 120:
            last_draw = now

            if oled is not None or not remote_busy():
                update_oled()

            service_io()


        handle_web()

        if not remote_busy():
            if sta.isconnected():
                ip = sta.ifconfig()[0]

                if ip != printed_ip:
                    printed_ip = ip
                    display_dirty = True
                    print('Mo web: http://' + ip)

            else:
                if printed_ip is not None:
                    printed_ip = None
                    display_dirty = True


        service_io()
        time.sleep_ms(1)

finally:
    web_ready = False
    remote_ready = False
    ir.irq(handler=None)
    set_fan(False)
    separate_command('q_off')
    gas_fan.value(G_OFF)
    set_buzzer(False)

    if BUZZER_PASSIVE:
        buzzer.deinit()

    for item in web_clients[:]:
        close_web_client(item)
    server.close()
    print('Da dung. Tat ca quat va coi da tat.')
