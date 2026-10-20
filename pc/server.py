"""Interface PC de SPY-BOT : serveur local entre le navigateur et l'ESP32.

    pip install pyserial
    python -m pc.server            # puis ouvrir http://localhost:8000
    python -m pc.server --lan      # accessible aussi depuis un téléphone du même Wi-Fi

Le navigateur affiche le robot en 3D et envoie les commandes ; ce serveur les
transmet à l'ESP32 par le port série USB (ou à un robot simulé, sans matériel)
et renvoie au navigateur tout ce que le robot répond.
"""

import argparse
import json
import os
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from hexapod import default_config  # noqa: E402
from hexapod.controller import Controller  # noqa: E402
from hexapod.servo import DryRunDriver  # noqa: E402

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
SIM = "sim"


class Hub:
    """Diffuse chaque ligne reçue du robot à tous les navigateurs connectés."""

    def __init__(self):
        self.clients = []
        self.lock = threading.Lock()
        self.last = {}  # dernière ligne par type, renvoyée aux nouveaux clients

    def publish(self, line):
        kind = line.split(" ", 1)[0]
        if kind in ("GEOM", "CAL", "LINK", "T"):
            self.last[kind] = line
        with self.lock:
            for q in self.clients:
                q.put(line)

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        for kind in ("LINK", "GEOM", "CAL", "T"):
            if kind in self.last:
                q.put(self.last[kind])
        with self.lock:
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.remove(q)


class SimLink:
    """Robot simulé : le même Controller que sur l'ESP32, avec des servos factices."""

    name = SIM

    def __init__(self, on_line):
        self.on_line = on_line
        calib = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_calib.json")
        self.ctrl = Controller(default_config(), DryRunDriver(), calib)
        self.inbox = queue.Queue()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def send(self, line):
        self.inbox.put(line)

    def close(self):
        self.running = False

    def _loop(self):
        time.sleep(0.1)  # comme l'ESP32 : HELLO une fois la liaison établie
        self.on_line("HELLO SPY-BOT (simulation)")
        last = time.monotonic()
        while self.running:
            while not self.inbox.empty():
                reply = self.ctrl.handle(self.inbox.get())
                if reply:
                    self.on_line(reply)
            now = time.monotonic()
            line = self.ctrl.tick(now - last)
            last = now
            if line:
                self.on_line(line)
            time.sleep(0.02)


class SerialLink:
    """ESP32 branché en USB, firmware main.py sous MicroPython."""

    def __init__(self, port, on_line, baudrate=115200):
        import serial  # pip install pyserial

        self.name = port
        self.on_line = on_line
        self.serial = serial.Serial(port, baudrate, timeout=0.2)
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._read, daemon=True).start()

    def send(self, line):
        with self.lock:
            self.serial.write((line + "\n").encode())

    def close(self):
        self.running = False
        try:
            self.serial.close()
        except Exception:
            pass

    def _read(self):
        while self.running:
            try:
                raw = self.serial.readline()
            except Exception as e:  # câble débranché...
                if self.running:
                    self.on_line("LINK error %s" % e)
                return
            line = raw.decode("utf-8", "replace").strip()
            if line:
                self.on_line(line)


RECONNECT_PERIOD = 2.0  # s entre deux tentatives quand le câble a sauté


class App:
    def __init__(self):
        self.hub = Hub()
        self.link = None
        self.lock = threading.Lock()
        self.wanted = None  # port à rouvrir tout seul si la liaison tombe

    def on_line(self, line):
        self.hub.publish(line)
        if line.startswith("HELLO"):
            # Le robot vient de démarrer : on récupère sa géométrie et ses trims.
            self.request_info()
        elif line.startswith("LINK error"):
            self._reconnect_later()

    def _reconnect_later(self):
        port = self.wanted
        with self.lock:
            if self.link:
                self.link.close()
                self.link = None

        def retry():
            while self.wanted == port and self.link is None:
                time.sleep(RECONNECT_PERIOD)
                if self.wanted != port:
                    return
                try:
                    with self.lock:
                        if self.wanted != port or self.link is not None:
                            return
                        self.link = SerialLink(port, self.on_line)
                except Exception:
                    continue
                self.hub.publish("LINK connected %s" % port)
                threading.Timer(2.5, self.request_info).start()

        if port and port != SIM:
            self.hub.publish("LINK reconnecting %s" % port)
            threading.Thread(target=retry, daemon=True).start()

    def request_info(self):
        try:
            self.send("GEOM")
            self.send("GETCAL")
        except RuntimeError:
            pass

    def connect(self, port):
        with self.lock:
            self._disconnect()
            self.wanted = port
            if port == SIM:
                self.link = SimLink(self.on_line)
            else:
                self.link = SerialLink(port, self.on_line)
                # Si la carte n'a pas redémarré à l'ouverture du port, pas de
                # HELLO : on demande quand même la géométrie.
                threading.Timer(2.5, self.request_info).start()
        self.hub.publish("LINK connected %s" % port)

    def disconnect(self):
        with self.lock:
            self.wanted = None
            self._disconnect()
        self.hub.publish("LINK disconnected")

    def _disconnect(self):
        if self.link:
            try:
                self.link.send("POWER 0")
            except Exception:
                pass
            self.link.close()
            self.link = None

    def send(self, line):
        link = self.link
        if link is None:
            raise RuntimeError("aucun robot connecté")
        link.send(line)


def lan_address():
    """Adresse IP de ce PC sur le réseau local (sans rien envoyer)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.0.1", 9))
        return s.getsockname()[0]
    except OSError:
        return "IP-du-PC"
    finally:
        s.close()


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    return [{"device": p.device, "description": p.description} for p in lp.comports()]


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, data, status=200):
            body = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return self.rfile.read(n).decode() if n else ""

        def do_GET(self):
            self.path = self.path.split("?", 1)[0]
            if self.path == "/events":
                return self._events()
            if self.path == "/ports":
                return self._json({"ports": list_ports(), "link": app.link.name if app.link else None})
            name = "index.html" if self.path in ("/", "") else self.path.lstrip("/")
            path = os.path.normpath(os.path.join(WEB, name))
            if not path.startswith(WEB) or not os.path.isfile(path):
                self.send_error(404)
                return
            types = {
                ".html": "text/html",
                ".js": "text/javascript",
                ".css": "text/css",
                ".webmanifest": "application/manifest+json",
                ".svg": "image/svg+xml",
                ".png": "image/png",
            }
            with open(path, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", types.get(os.path.splitext(path)[1], "application/octet-stream") + "; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            try:
                if self.path == "/cmd":
                    for line in self._body().splitlines():
                        if line.strip():
                            app.send(line.strip())
                elif self.path == "/connect":
                    app.connect(json.loads(self._body())["port"])
                elif self.path == "/disconnect":
                    app.disconnect()
                else:
                    return self.send_error(404)
            except Exception as e:
                return self._json({"error": str(e)}, 400)
            self._json({"ok": True})

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = app.hub.subscribe()
            try:
                while True:
                    try:
                        line = q.get(timeout=5)
                    except queue.Empty:
                        line = None
                    msg = ": ping\n\n" if line is None else "data: %s\n\n" % line
                    self.wfile.write(msg.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                app.hub.unsubscribe(q)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="port série de l'ESP32 (ex. COM3, /dev/ttyUSB0) ou 'sim'")
    parser.add_argument("--http", type=int, default=8000, help="port de l'interface web")
    parser.add_argument(
        "--lan",
        action="store_true",
        help="ouvrir l'interface aux autres appareils du réseau (téléphone) ; "
        "toute personne sur ce Wi-Fi peut alors piloter le robot",
    )
    args = parser.parse_args()

    app = App()
    if args.port:
        app.connect(args.port)
    server = ThreadingHTTPServer(("0.0.0.0" if args.lan else "127.0.0.1", args.http), make_handler(app))
    print("Interface SPY-BOT : http://localhost:%d" % args.http)
    if args.lan:
        print("Depuis le téléphone (même Wi-Fi) : http://%s:%d" % (lan_address(), args.http))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.disconnect()


if __name__ == "__main__":
    main()
