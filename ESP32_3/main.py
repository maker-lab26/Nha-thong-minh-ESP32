import uasyncio as asyncio
import gc
import os
print("NOVA SERVO HOLD V4 STREAM - ESP32 SO 3 (RAIN AUTO)")

import _thread
import socket
import json
import time

from machine import ADC, PWM, Pin, time_pulse_us
from config import WIFI_SSID, WIFI_PASSWORD, CONTROL_KEY
from auto_network import AutoNetwork, LocalName


PIN_POWER_RAIN = 23
PIN_ADC_RAIN = 36
PIN_SERVO = 19

ANGLE_OFF = 0
ANGLE_ON = 90


RAIN_INVERT = True


if RAIN_INVERT:
    THRESHOLD_RAIN = 680
    THRESHOLD_DRY = 600
else:
    THRESHOLD_RAIN = 600
    THRESHOLD_DRY = 680

RAIN_STABLE_SAMPLES = 3
RAIN_SAMPLE_MS = 150


ADC_NO_SIGNAL = 5


SERVO_MOVE_MS = 1200


SERVO_MIN_US = 600
SERVO_MAX_US = 2400


SERVO_MIN_CHANGE_MS = 4000


SERVO_NOISE_GUARD_MS = 300


SERVO_BLOCK_ULTRASONIC = True


ULTRASONIC_CONFIRM_READS = 1


SERVO_RELAX_AFTER_MOVE = False


ADC_SAMPLES = 8

LED_PINS = [2, 4, 5, 18]

leds = [Pin(number, Pin.OUT) for number in LED_PINS]
led_states = [False, False, False, False]

for led in leds:
    led.value(0)

trig1 = Pin(12, Pin.OUT)
echo1 = Pin(13, Pin.IN)

trig2 = Pin(14, Pin.OUT)
echo2 = Pin(27, Pin.IN)

trig1.value(0)
trig2.value(0)


DISTANCE_THRESHOLD = 10


ECHO_TIMEOUT_US = 25000
people_count = 0

state_guard = _thread.allocate_lock()


power_pin = Pin(PIN_POWER_RAIN, Pin.OUT)
power_pin.value(1)

rain_adc = ADC(Pin(PIN_ADC_RAIN))
rain_adc.atten(ADC.ATTN_11DB)


class RoofController:

    def __init__(self, adc, pwm):
        self.adc = adc
        self.pwm = pwm
        self.pwm.duty(0)

        self.guard = _thread.allocate_lock()

        self.raw = None
        self.valid = False
        self.raining = False
        self.dry_samples = 0


        self.rain_auto = True
        self.th_rain = THRESHOLD_RAIN
        self.th_dry = THRESHOLD_DRY

        self.requested_on = False
        self.target = ANGLE_ON
        self.angle = -1
        self.resend = True

        self.stop_at = None
        self.last_move_at = None
        self.quiet_until = None
        self.sample_at = time.ticks_ms()

        self.worker_ok = False
        self.last_error_at = None

    def update_target(self):

        if self.rain_auto and (self.raining or not self.valid):
            self.target = ANGLE_ON
        else:
            self.target = ANGLE_ON if self.requested_on else ANGLE_OFF

    def command(self, action, payload=b""):
        self.guard.acquire()

        try:
            if action in ("roof_on", "roof_90", "roof_close"):
                self.requested_on = True

            elif action in ("roof_off", "roof_0", "roof_open"):
                self.requested_on = False

            elif action == "rain_auto_on":
                self.rain_auto = True

            elif action == "rain_auto_off":
                self.rain_auto = False

            elif action == "rain_auto_toggle":
                self.rain_auto = not self.rain_auto

            elif action == "rain_threshold":

                data = json.loads(payload.decode())
                rain = int(data["rain"])
                dry = int(data["dry"])
                if RAIN_INVERT:
                    if not (0 <= dry < rain <= 4095):
                        raise ValueError("Threshold must be 0 <= dry < rain <= 4095")
                else:
                    if not (0 < rain < dry <= 4095):
                        raise ValueError("Threshold must be 0 < rain < dry <= 4095")
                self.th_rain = rain
                self.th_dry = dry
                self.dry_samples = 0

            else:
                raise ValueError("Unknown roof command")


            self.update_target()
            self.resend = True

            print(
                "[MAI CHE]",
                action,
                "target:",
                self.target,
                "rain_auto:",
                self.rain_auto,
                "rain:",
                self.raining
            )

        finally:
            self.guard.release()

    def servo_noisy(self):

        if not SERVO_BLOCK_ULTRASONIC:
            return False
        self.guard.acquire()
        try:
            if self.quiet_until is None:
                return False
            if time.ticks_diff(time.ticks_ms(), self.quiet_until) >= 0:
                self.quiet_until = None
                return False
            return True
        finally:
            self.guard.release()

    def _write_pwm(self, pulse_us):

        if pulse_us is None:
            pulse_us = 0

        if hasattr(self.pwm, "duty_ns"):
            self.pwm.duty_ns(pulse_us * 1000)
        elif hasattr(self.pwm, "duty_u16"):
            self.pwm.duty_u16(pulse_us * 65535 // 20000)
        else:
            self.pwm.duty(pulse_us * 1023 // 20000)

    def tick(self):
        self.guard.acquire()

        try:
            now = time.ticks_ms()

            if self.stop_at is not None:
                if time.ticks_diff(now, self.stop_at) >= 0:
                    self.stop_at = None
                    if SERVO_RELAX_AFTER_MOVE:

                        self._write_pwm(None)


            if time.ticks_diff(now, self.sample_at) >= 0:
                self.sample_at = time.ticks_add(
                    now, RAIN_SAMPLE_MS
                )

                try:
                    total = 0
                    for _ in range(ADC_SAMPLES):
                        total += self.adc.read()
                    self.raw = total // ADC_SAMPLES


                    if not RAIN_INVERT and self.raw <= ADC_NO_SIGNAL:
                        raise OSError("Rain ADC = 0, kiem tra day AO/GPIO36")


                    if RAIN_INVERT:
                        is_wet = self.raw >= self.th_rain
                        is_dry = self.raw < self.th_dry
                    else:
                        is_wet = self.raw < self.th_rain
                        is_dry = self.raw >= self.th_dry

                    if is_wet:
                        self.raining = True
                        self.valid = True
                        self.dry_samples = 0

                    elif is_dry:
                        self.dry_samples = min(
                            self.dry_samples + 1,
                            RAIN_STABLE_SAMPLES
                        )

                        if self.dry_samples >= RAIN_STABLE_SAMPLES:
                            self.raining = False
                            self.valid = True

                    else:
                        self.dry_samples = 0

                except Exception as error:
                    self.valid = False
                    self.raining = False
                    self.dry_samples = 0

                    if (
                        self.last_error_at is None
                        or time.ticks_diff(
                            now, self.last_error_at
                        ) >= 5000
                    ):
                        print("[RAIN ADC]", error)
                        self.last_error_at = now

            self.update_target()


            settled = (
                self.last_move_at is None
                or time.ticks_diff(now, self.last_move_at) >= SERVO_MIN_CHANGE_MS
            )

            if self.resend or (self.target != self.angle and settled):
                pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * self.target // 180
                self._write_pwm(pulse_us)

                if self.target != self.angle:
                    self.last_move_at = now
                    self.quiet_until = time.ticks_add(
                        now, SERVO_MOVE_MS + SERVO_NOISE_GUARD_MS
                    )
                self.stop_at = time.ticks_add(now, SERVO_MOVE_MS)

                self.resend = False
                print('[SERVO] GPIO', PIN_SERVO, 'goc lenh:', self.target,
                      'xung us:', pulse_us, 'rain ADC:', self.raw,
                      'valid:', self.valid, 'rain:', self.raining,
                      'auto:', self.rain_auto)
                self.angle = self.target

            self.worker_ok = True

        finally:
            self.guard.release()

    def snapshot(self):
        self.guard.acquire()

        try:
            forced = self.rain_auto and (self.raining or not self.valid)

            return {
                "rain_raw": self.raw,
                "rain": self.raining,
                "rain_valid": self.valid,
                "rain_error": not self.valid,
                "rain_invert": RAIN_INVERT,


                "rain_auto": self.rain_auto,
                "th_rain": self.th_rain,
                "th_dry": self.th_dry,

                "angle": self.angle,
                "roof_target": self.target,

                "roof_switch_control": True,
                "roof_requested_on": self.requested_on,
                "roof_on": self.target == ANGLE_ON,

                "roof_forced90": forced,
                "roof_rain_override": self.rain_auto and self.raining,

                "roof_moving": (
                    self.stop_at is not None
                    or self.angle != self.target
                ),

                "roof_worker_ok": self.worker_ok,
                "servo_hold": True,
                "servo_pin": PIN_SERVO,
                "angle_is_command": True
            }

        finally:
            self.guard.release()


roof = RoofController(
    rain_adc,
    PWM(Pin(PIN_SERVO), freq=50)
)


def get_distance(trig, echo):
    trig.value(0)
    time.sleep_us(2)

    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    try:
        duration = time_pulse_us(echo, 1, ECHO_TIMEOUT_US)

        if duration <= 0:
            return 999.0

        return duration / 58.0

    except Exception:
        return 999.0


last_dist = [999.0, 999.0]


def is_near(trig, echo, idx):
    d = get_distance(trig, echo)
    last_dist[idx] = d
    if d >= DISTANCE_THRESHOLD:
        return False

    for _ in range(ULTRASONIC_CONFIRM_READS - 1):
        time.sleep_ms(30)
        if get_distance(trig, echo) >= DISTANCE_THRESHOLD:
            return False
    return True


def people_counter_thread():
    global people_count

    phase = "idle"
    started = time.ticks_ms()
    prev_count = 0
    last_log = time.ticks_ms()

    print("[OK] Luong servo + dem nguoi da chay")

    while True:
        try:
            roof.tick()

            if roof.servo_noisy():


                phase = "clear"
                if time.ticks_diff(time.ticks_ms(), last_log) >= 1000:
                    print("[SIEU AM] TAM DUNG: servo dang quay")
                    last_log = time.ticks_ms()
                time.sleep_ms(50)
                continue

            s1 = is_near(trig1, echo1, 0)

            time.sleep_ms(60)

            s2 = is_near(trig2, echo2, 1)

            now = time.ticks_ms()

            state_guard.acquire()

            try:
                if phase == "idle":
                    if s1 and s2:
                        phase = "clear"

                    elif s1:
                        phase = "first1"
                        started = now

                    elif s2:
                        phase = "first2"
                        started = now

                elif phase == "first1":
                    if time.ticks_diff(now, started) > 5000:
                        phase = "clear"

                    elif s2:
                        people_count += 1
                        phase = "clear"

                elif phase == "first2":
                    if time.ticks_diff(now, started) > 5000:
                        phase = "clear"

                    elif s1:
                        people_count = max(0, people_count - 1)
                        phase = "clear"

                elif not s1 and not s2:
                    phase = "idle"

                if people_count != prev_count:
                    value = people_count > 0

                    for i in range(4):
                        led_states[i] = value
                        leds[i].value(1 if value else 0)

                    prev_count = people_count

            finally:
                state_guard.release()

            if time.ticks_diff(now, last_log) >= 1000:
                print("[SIEU AM] S1:", round(last_dist[0], 1),
                      "cm | S2:", round(last_dist[1], 1),
                      "cm | GAN:", s1, s2, "| PHA:", phase,
                      "| DEM:", people_count)
                last_log = now
            roof.tick()
            time.sleep_ms(60)

        except Exception as error:
            print("[WORKER]", error)

            roof.guard.acquire()
            try:
                roof.worker_ok = False
            finally:
                roof.guard.release()

            time.sleep_ms(100)


auto_net = AutoNetwork(
    3,
    WIFI_SSID,
    WIFI_PASSWORD,
    CONTROL_KEY
)

wifi = auto_net.sta

try:
    wifi.config(pm=0)
except (ValueError, TypeError, OSError):
    pass

local_name = LocalName(wifi)


NODE1_ACTIONS = (
    "toggle", "up", "down", "fan_on", "fan_off", "ac_on",
    "ac_off", "auto", "manual", "q_toggle", "q_on", "q_off",
    "gas_mute", "gas_unmute", "gas_threshold"
)
NODE2_ACTIONS = (
    "open", "lock", "password", "rfid_list", "rfid_scan",
    "rfid_status", "rfid_add", "rfid_remove", "rfid_replace"
)
ROOF_ACTIONS = (
    "roof_on", "roof_off", "roof_0", "roof_90",
    "roof_open", "roof_close",
    "rain_auto_on", "rain_auto_off", "rain_auto_toggle",
    "rain_threshold"
)
node_locks = {1: asyncio.Lock(), 2: asyncio.Lock()}
cache = {1: None, 2: None}
cache_at = {1: None, 2: None}
cache_epoch = {1: 0, 2: 0}
command_pending = {1: False, 2: False}
CACHE_MAX_AGE_MS = 4500
active_clients = 0
MAX_CLIENTS = 4
MAX_HEADER = 2048
MAX_BODY = 1024


READ_TIMEOUT_MS = 3000


def local_state():
    state = roof.snapshot()
    state_guard.acquire()
    try:
        state.update({"count": people_count, "leds": led_states[:]})
    finally:
        state_guard.release()
    state["us_blocked"] = roof.servo_noisy()
    state["dist1"] = round(last_dist[0], 1)
    state["dist2"] = round(last_dist[1], 1)
    state["ip"] = wifi.ifconfig()[0]
    state["firmware"] = "NOVA_ALL_FAST_2"
    return state


async def close_stream(writer):
    if writer is None:
        return
    try:
        writer.close()
        await asyncio.wait_for_ms(writer.wait_closed(), 300)
    except Exception:
        pass


async def reply(writer, code, body, mime="application/json; charset=utf-8"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 %d Response\r\n" % code
        + "Content-Type: " + mime + "\r\n"
        + "Content-Length: %d\r\n" % len(body)
        + "Cache-Control: no-store\r\n"
        + "Connection: close\r\n\r\n"
    ).encode()
    writer.write(header)
    await writer.drain()
    for offset in range(0, len(body), 1024):
        writer.write(body[offset:offset + 1024])
        await writer.drain()


async def reply_index(writer):

    gc.collect()
    try:
        size = os.stat('index.html')[6]
        file = open('index.html', 'rb')
    except OSError:
        await reply(writer, 404, '{"error":"Missing index.html"}')
        return
    try:
        writer.write(('HTTP/1.1 200 OK\r\n'
                      'Content-Type: text/html; charset=utf-8\r\n'
                      'Content-Length: %d\r\n'
                      'Cache-Control: no-store\r\n'
                      'Connection: close\r\n\r\n' % size).encode())
        await writer.drain()
        while True:
            chunk = file.read(512)
            if not chunk:
                break
            writer.write(chunk)
            await writer.drain()
            await asyncio.sleep_ms(0)
    finally:
        file.close()
        gc.collect()


async def read_request(reader):
    raw = b""
    while b"\r\n\r\n" not in raw:
        chunk = await reader.read(512)
        if not chunk:
            raise ValueError("Incomplete request")
        raw += chunk
        if len(raw) > MAX_HEADER + MAX_BODY + 4:
            raise ValueError("Request too large")
    header, body = raw.split(b"\r\n\r\n", 1)
    if len(header) > MAX_HEADER:
        raise ValueError("Header too large")
    lines = header.decode().split("\r\n")
    method, path, version = lines[0].split()
    if version not in ("HTTP/1.0", "HTTP/1.1"):
        raise ValueError("HTTP version")
    headers = {}
    for line in lines[1:]:
        key, value = line.split(":", 1)
        key = key.strip().lower()
        if not key or key in headers:
            raise ValueError("Invalid header")
        headers[key] = value.strip()
    if "transfer-encoding" in headers:
        raise ValueError("Unsupported encoding")
    length_text = headers.get("content-length", "0")
    if not length_text or any(c not in "0123456789" for c in length_text):
        raise ValueError("Invalid body length")
    length = int(length_text)
    if length > MAX_BODY or len(body) > length:
        raise ValueError("Body too large")
    while len(body) < length:
        chunk = await reader.read(length - len(body))
        if not chunk:
            raise ValueError("Incomplete body")
        body += chunk
    return method, path.split("?", 1)[0], body


async def forward_inner(node, method, path, payload):
    writer = None
    async with node_locks[node]:
        try:
            ip = auto_net.address(node)
            if not ip:
                raise OSError("Board address unavailable")
            reader, writer = await asyncio.open_connection(ip, 80)
            head = (
                method + " " + path + " HTTP/1.0\r\n"
                + "Host: " + ip + "\r\n"
                + "X-Control-Key: " + CONTROL_KEY + "\r\n"
                + "Content-Type: application/json\r\n"
                + "Content-Length: %d\r\n" % len(payload)
                + "Connection: close\r\n\r\n"
            ).encode()
            writer.write(head)
            if payload:
                writer.write(payload)
            await writer.drain()
            raw = b""
            while b"\r\n\r\n" not in raw:
                chunk = await reader.read(512)
                if not chunk:
                    raise OSError("Incomplete board headers")
                raw += chunk
                if len(raw) > 4096:
                    raise OSError("Board headers too large")
            head, body = raw.split(b"\r\n\r\n", 1)
            code = int(head.split(b"\r\n", 1)[0].split()[1])
            length = None
            for line in head.split(b"\r\n")[1:]:
                if line.lower().startswith(b"transfer-encoding:"):
                    raise OSError("Unsupported board encoding")
                if line.lower().startswith(b"content-length:"):
                    length = int(line.split(b":", 1)[1].strip())
            if length is not None and not 0 <= length <= 8192:
                raise OSError("Board response too large")
            while length is None or len(body) < length:
                remaining = 512 if length is None else min(512, length-len(body))
                chunk = await reader.read(remaining)
                if not chunk:
                    break
                body += chunk
                if len(body) > 8192:
                    raise OSError("Board response too large")
            if length is not None:
                if len(body) < length:
                    raise OSError("Incomplete board body")
                body = body[:length]
            return code, body
        finally:
            await close_stream(writer)


async def forward(node, method, path, payload=b""):

    return await asyncio.wait_for_ms(
        forward_inner(node, method, path, payload),
        (4000 if method == "POST" else 1400)
    )


def cached_node(node):
    at = cache_at[node]
    age = None if at is None else max(0, time.ticks_diff(time.ticks_ms(), at))
    return {
        "online": age is not None and age <= CACHE_MAX_AGE_MS,
        "age_ms": age,
        "data": cache[node]
    }


def update_cache(node, body):
    data = json.loads(body)
    if not isinstance(data, dict):
        raise ValueError("Invalid board state")

    if node == 1 and "target" not in data:
        return
    if node == 2 and "open" not in data:
        return
    cache[node] = data
    cache_at[node] = time.ticks_ms()


async def poll_remote(node):
    failures = 0
    while True:
        if command_pending[node]:
            await asyncio.sleep_ms(50)
            continue
        epoch = cache_epoch[node]
        try:
            code, body = await forward(node, "GET", "/api")
            if code != 200:
                raise OSError("Board status error")
            if epoch == cache_epoch[node]:
                update_cache(node, body)
            failures = 0
        except Exception:
            failures = min(4, failures + 1)
        delay = 800 if failures == 0 else min(5000, 500 * (2 ** (failures - 1)))
        await asyncio.sleep_ms(delay)


async def route(writer, method, path, payload):
    global people_count
    if method == "GET" and path in ("/", "/index.html"):
        await reply_index(writer)
        return
    if method == "GET" and path == "/api/all":
        snapshot = {"1": cached_node(1), "2": cached_node(2),
                    "3": {"online": True, "age_ms": 0, "data": local_state()}}
        await reply(writer, 200, json.dumps({"nodes": snapshot}))
        return
    if method == "GET" and path == "/api/3":
        await reply(writer, 200, json.dumps(local_state()))
        return
    if method == "GET" and path in ("/api/1", "/api/2"):
        entry = cached_node(int(path[-1]))
        if entry["online"]:
            await reply(writer, 200, json.dumps(entry["data"]))
        else:
            await reply(writer, 503, '{"error":"Board offline or timeout"}')
        return
    if method == "POST" and path.startswith("/cmd/"):
        parts = path.strip("/").split("/")
        if len(parts) != 3:
            await reply(writer, 400, '{"error":"Bad command"}')
            return
        node, action = parts[1:]
        if node in ("1", "2"):
            allowed = NODE1_ACTIONS if node == "1" else NODE2_ACTIONS
            if action not in allowed:
                await reply(writer, 400, '{"error":"Unknown command"}')
                return
            number = int(node)
            if command_pending[number]:
                await reply(writer, 409, '{"error":"Command already in progress"}')
                return
            command_pending[number] = True
            cache_epoch[number] += 1
            try:
                code, body = await forward(number, "POST", "/cmd/" + action, payload)
                if code == 200:
                    update_cache(number, body)
                await reply(writer, code, body)
            finally:
                command_pending[number] = False
            return
        if node == "3":
            if action in ROOF_ACTIONS:
                roof.command(action, payload)
            elif action == "reset":
                state_guard.acquire()
                try:
                    people_count = 0
                    for i in range(4):
                        led_states[i] = False
                        leds[i].value(0)
                finally:
                    state_guard.release()
            elif (len(action) == 5 and action.startswith("led")
                  and action[3] in "0123" and action[4] in "01"):
                i = int(action[3])
                value = action[4] == "1"
                state_guard.acquire()
                try:
                    led_states[i] = value
                    leds[i].value(1 if value else 0)
                finally:
                    state_guard.release()
            else:
                await reply(writer, 400, '{"error":"Unknown command"}')
                return
            await reply(writer, 200, json.dumps(local_state()))
            return
    await reply(writer, 404, '{"error":"Not found"}')


async def handle_client(reader, writer):
    global active_clients
    if active_clients >= MAX_CLIENTS:
        try:
            await asyncio.wait_for_ms(
                reply(writer, 503, '{"error":"Hub busy"}'), 500
            )
        except Exception:
            pass
        await close_stream(writer)
        return
    active_clients += 1
    stage = "read"
    try:
        method, path, payload = await asyncio.wait_for_ms(
            read_request(reader), READ_TIMEOUT_MS
        )
        stage = method + " " + path
        await asyncio.wait_for_ms(route(writer, method, path, payload),
                                  15000 if path in ("/", "/index.html") else 5000)
    except Exception as error:
        name = type(error).__name__
        if stage == "read" and (name == "TimeoutError"
                                or str(error) == "Incomplete request"):

            pass
        else:

            print("[HTTP]", stage, name, str(error))
            try:
                code = 400 if isinstance(error, ValueError) else 503
                message = "Bad request" if code == 400 else "Board offline or timeout"
                await asyncio.wait_for_ms(
                    reply(writer, code, json.dumps({"error": message})), 500
                )
            except Exception:
                pass
    finally:
        await close_stream(writer)
        active_clients -= 1


async def network_service():
    printed = None
    gc_at = time.ticks_ms()
    while True:
        try:
            auto_net.tick()
            local_name.tick()
            if wifi.isconnected():
                ip = wifi.ifconfig()[0]
                if ip != printed:
                    print("[WEB] http://" + ip)
                    printed = ip
            if time.ticks_diff(time.ticks_ms(), gc_at) >= 10000:
                gc.collect()
                gc_at = time.ticks_ms()
        except Exception as error:
            print("[NETWORK]", error)
        await asyncio.sleep_ms(100)


async def main():
    _thread.start_new_thread(people_counter_thread, ())
    server = await asyncio.start_server(handle_client, "0.0.0.0", 80, backlog=8)
    asyncio.create_task(network_service())
    asyncio.create_task(poll_remote(1))
    asyncio.create_task(poll_remote(2))
    print("[OK] NOVA SERVO HOLD V4 STREAM - servo GPIO19 0/90")
    try:
        while True:
            await asyncio.sleep(60)
    finally:
        server.close()
        await server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        asyncio.new_event_loop()
