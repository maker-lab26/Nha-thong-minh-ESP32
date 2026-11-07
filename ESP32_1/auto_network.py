
import network
import socket
import time
import json

DISCOVERY_PORT = 37020
PEER_TTL_MS = 30000


class AutoNetwork:
    def __init__(self, node, ssid, password, key):
        self.node = node
        self.ssid = ssid
        self.password = password
        self.key = key
        self.peers = {}
        self.last_ip = None
        now = time.ticks_ms()
        self.last_attempt = time.ticks_add(now, -15000)
        self.last_search = time.ticks_add(now, -3000)
        self.last_announce = time.ticks_add(now, -5000)
        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)
        try:
            self.sta.config(pm=getattr(network.WLAN, 'PM_NONE', 0))
        except (AttributeError, ValueError, TypeError, OSError):
            pass
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            except (AttributeError, OSError):
                pass
            udp.bind(('0.0.0.0', DISCOVERY_PORT))
            udp.setblocking(False)
        except Exception:
            udp.close()
            raise
        self.udp = udp
        self.tick()

    def message(self, kind):
        return json.dumps({'app': 'NOVA2', 'kind': kind,
                           'node': self.node, 'key': self.key}).encode()

    def broadcast(self, kind):
        try:
            self.udp.sendto(self.message(kind),
                            ('255.255.255.255', DISCOVERY_PORT))
        except OSError:
            pass

    def tick(self):
        now = time.ticks_ms()
        if not self.sta.isconnected():
            if self.last_ip is not None:
                self.peers.clear()
            self.last_ip = None
            if time.ticks_diff(now, self.last_attempt) >= 15000:
                self.last_attempt = now
                try:
                    
                    self.sta.disconnect()
                    self.sta.connect(self.ssid, self.password)
                except OSError as error:
                    print('[WIFI]', error)
            return
        ip = self.sta.ifconfig()[0]
        if ip != self.last_ip:
            self.last_ip = ip
            self.peers.clear()
            self.last_search = time.ticks_add(now, -3000)
            self.last_announce = time.ticks_add(now, -5000)
            print('[NOVA] BOARD', self.node, 'IP:', ip)
            if self.node == 3:
                print('[WEB] http://' + ip)
        if self.node == 3:
            if time.ticks_diff(now, self.last_search) >= 3000:
                self.last_search = now
                self.broadcast('find')
        elif time.ticks_diff(now, self.last_announce) >= 5000:
            self.last_announce = now
            self.broadcast('here')
        
        for _ in range(8):
            try:
                data, addr = self.udp.recvfrom(512)
            except OSError:
                break
            try:
                obj = json.loads(data)
                if not isinstance(obj, dict):
                    continue
                if obj.get('app') != 'NOVA2' or obj.get('key') != self.key:
                    continue
                node = obj.get('node')
                if obj.get('kind') == 'find' and node == 3 and self.node != 3:
                    self.udp.sendto(self.message('here'), addr)
                elif obj.get('kind') == 'here' and self.node == 3 and node in (1, 2):
                    previous = self.peers.get(node)
                    self.peers[node] = (addr[0], now)
                    if previous is None or previous[0] != addr[0]:
                        print('[FOUND]', node, addr[0])
            except (ValueError, TypeError, OSError):
                pass

    def address(self, node):
        
        peer = self.peers.get(node)
        if peer and time.ticks_diff(time.ticks_ms(), peer[1]) < PEER_TTL_MS:
            return peer[0]
        raise OSError('Chua tim thay ESP32 ' + str(node))

    def close(self):
        self.udp.close()


class LocalName:
    def __init__(self, sta, label='nova'):
        self.sta = sta
        self.name = label + '.local'
        self.wire = bytes([len(label)]) + label.encode() + b'\x05local\x00'
        self.sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock = sock
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', 5353))
            sock.setsockopt(socket.IPPROTO_IP,
                            getattr(socket, 'IP_ADD_MEMBERSHIP', 3),
                            bytes([224, 0, 0, 251, 0, 0, 0, 0]))
            sock.setblocking(False)
        except (OSError, AttributeError) as error:
            print('[mDNS] Dung IP:', error)
            if self.sock:
                self.sock.close()
            self.sock = None

    def read_name(self, data, start):
        labels = []
        end = None
        pos = start
        for _ in range(32):
            size = data[pos]
            if size == 0:
                return '.'.join(labels).lower(), end if end is not None else pos + 1
            if size & 192 == 192:
                if end is None:
                    end = pos + 2
                pos = ((size & 63) << 8) | data[pos + 1]
            elif size <= 63:
                if pos + 1 + size > len(data):
                    raise ValueError('Truncated DNS name')
                labels.append(data[pos + 1:pos + 1 + size].decode())
                pos += 1 + size
            else:
                raise ValueError('Invalid DNS label')
        raise ValueError('DNS name loop')

    def tick(self):
        if not self.sock or not self.sta.isconnected():
            return
        for _ in range(2):
            try:
                data, addr = self.sock.recvfrom(1024)
            except OSError:
                return
            try:
                if len(data) < 12 or data[2] & 128:
                    continue
                count = (data[4] << 8) | data[5]
                pos = 12
                for _ in range(min(count, 16)):
                    name, pos = self.read_name(data, pos)
                    typ = (data[pos] << 8) | data[pos + 1]
                    cls = (data[pos + 2] << 8) | data[pos + 3]
                    pos += 4
                    if name == self.name and typ in (1, 255) and cls & 32767 == 1:
                        ip = bytes(int(v) for v in self.sta.ifconfig()[0].split('.'))
                        record = self.wire + b'\x00\x01\x80\x01\x00\x00\x00\x0a\x00\x04' + ip
                        if addr[1] != 5353:
                            question = self.wire + b'\x00\x01\x00\x01'
                            packet = data[:2] + b'\x84\x00\x00\x01\x00\x01\x00\x00\x00\x00' + question + record
                            self.sock.sendto(packet, addr)
                        else:
                            packet = b'\x00\x00\x84\x00\x00\x00\x00\x01\x00\x00\x00\x00' + record
                            self.sock.sendto(packet, addr if cls & 32768 else ('224.0.0.251', 5353))
                        break
            except (IndexError, ValueError, UnicodeError, OSError):
                pass
