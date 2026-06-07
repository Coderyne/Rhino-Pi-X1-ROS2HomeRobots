#!/usr/bin/env python3
"""MJPEG HTTP stream server — single producer, broadcast to N consumers.

Architecture:
  One camera-read thread produces frames → Condition.notify_all()
  Each stream client waits on the condition and writes the same frame.
  Bandwidth = 1× camera bitrate regardless of client count.

Two modes:
  --native   Passthrough: raw MJPG from camera → wire.  Zero CPU, high bitrate.
  (default)  Re-encode: decode MJPG → BGR → re-encode JPEG at --quality.
             Slightly more CPU, much lower bitrate.
"""

import argparse
import http.server
import json
import socketserver
import sys
import threading
import time

import cv2
import numpy as np


class CameraStream:
    def __init__(self, device, width, height, max_fps, fourcc_code, native=False, jpeg_quality=75):
        self.device = device
        self.width = width
        self.height = height
        self.max_fps = max_fps
        self.fourcc_code = fourcc_code
        self.native = native
        self.jpeg_quality = jpeg_quality
        self.cap = None
        self.status = "offline"
        self.last_frame = None
        self.placeholder = None
        self._seq = 0
        self._cond = threading.Condition()
        self._stop = False
        self._reconnect_interval = 5

    def _generate_placeholder(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:] = (30, 30, 40)

        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "Camera Offline"
        font_scale = 1.6
        thickness = 3
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
        x = (self.width - tw) // 2
        y = (self.height + th) // 2
        cv2.putText(img, text, (x, y), font, font_scale, (200, 200, 210), thickness, cv2.LINE_AA)

        (iw, ih), _ = cv2.getTextSize("(o)", font, 2.5, 2)
        ix = (self.width - iw) // 2
        iy = (self.height - th - ih) // 2 + 30
        cv2.putText(img, "(o)", (ix, iy), font, 2.5, (120, 120, 130), 2, cv2.LINE_AA)

        subtitle = "Check USB connection"
        (sw, _), _ = cv2.getTextSize(subtitle, font, 0.8, 1)
        sx = (self.width - sw) // 2
        sy = y + th + 40
        cv2.putText(img, subtitle, (sx, sy), font, 0.8, (120, 120, 130), 1, cv2.LINE_AA)

        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes()

    def _try_open(self):
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            cap.release()
            return None

        cap.set(cv2.CAP_PROP_FOURCC, self.fourcc_code)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.max_fps)

        if self.native:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            ret, buf = cap.read()
            if not ret or buf is None:
                cap.release()
                return None
        else:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            ret, frame = cap.read()
            if not ret or frame is None:
                cap.release()
                return None

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        mode_str = "native MJPG" if self.native else f"re-encode q={self.jpeg_quality}"
        print(f"[camera] Opened {self.device} — {actual_w}x{actual_h} @ {actual_fps:.1f}fps ({mode_str})")

        return cap

    def start(self):
        self.placeholder = self._generate_placeholder()
        with self._cond:
            self.last_frame = self.placeholder
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        frame_interval = 1.0 / self.max_fps
        last_time = 0.0

        while not self._stop:
            now = time.monotonic()
            if now - last_time < frame_interval:
                time.sleep(0.005)
                continue

            if self.cap is None:
                cap = self._try_open()
                if cap is not None:
                    self.cap = cap
                    self.status = "ok"
                else:
                    if self.status != "offline":
                        print(f"[camera] Cannot open {self.device}, retrying every {self._reconnect_interval}s",
                              file=sys.stderr)
                    self.status = "offline"
                    with self._cond:
                        self.last_frame = self.placeholder
                        self._seq += 1
                        self._cond.notify_all()
                    time.sleep(self._reconnect_interval)
                    last_time = time.monotonic()
                    continue

            try:
                if self.native:
                    ret, buf = self.cap.read()
                    if not ret or buf is None:
                        raise RuntimeError("native read failed")
                    jpeg = buf.tobytes()
                else:
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        raise RuntimeError("frame read failed")
                    _, jpeg = cv2.imencode('.jpg', frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    jpeg = jpeg.tobytes()

                with self._cond:
                    self.last_frame = jpeg
                    self._seq += 1
                    self._cond.notify_all()

                last_time = now
            except Exception as e:
                print(f"[camera] Error: {e}", file=sys.stderr)
                if self.cap:
                    self.cap.release()
                self.cap = None
                self.status = "offline"

    def wait_frame(self, last_seq):
        with self._cond:
            while self._seq == last_seq and not self._stop:
                self._cond.wait(timeout=0.5)
            return self.last_frame, self._seq

    def snapshot(self):
        with self._cond:
            return self.last_frame

    def get_status(self):
        return {"status": self.status, "fps": self.max_fps, "resolution": f"{self.width}x{self.height}"}

    def stop(self):
        self._stop = True
        with self._cond:
            self._cond.notify_all()
        if self.cap:
            self.cap.release()


class MJPEGHandler(http.server.BaseHTTPRequestHandler):
    camera_stream = None
    log_requests = True

    def log_message(self, fmt, *args):
        if not MJPEGHandler.log_requests:
            return
        super().log_message(fmt, *args)

    def do_GET(self):
        if self.path == '/stream':
            self._handle_stream()
        elif self.path == '/status':
            self._handle_status()
        elif self.path == '/snapshot':
            self._handle_snapshot()
        elif self.path == '/':
            self._200_text(b'MJPEG Camera Stream Server\n/stream /status /snapshot\n')
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        self._cors()
        self.send_response(200)
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.end_headers()

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')

    def _200_text(self, body):
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_stream(self):
        cs = self.camera_stream
        if cs is None:
            self.send_response(503)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Connection', 'keep-alive')
        self._cors()
        self.end_headers()

        try:
            self.wfile.flush()
        except Exception:
            pass

        seq = -1
        while True:
            frame, seq = cs.wait_frame(seq)
            if frame is None:
                continue
            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                break

    def _handle_status(self):
        cs = self.camera_stream
        status = cs.get_status() if cs else {"status": "offline"}
        self._send_json(status)

    def _handle_snapshot(self):
        cs = self.camera_stream
        if cs is None:
            self.send_response(503)
            self.end_headers()
            return
        frame = cs.snapshot()
        if frame is None:
            self.send_response(503)
            self._200_text(b'Camera offline')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self._cors()
        self.end_headers()
        self.wfile.write(frame)

    def _send_json(self, obj):
        data = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self._cors()
        self.end_headers()
        self.wfile.write(data)


class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser(description='MJPEG Camera Stream Server')
    parser.add_argument('--port', type=int, default=8193)
    parser.add_argument('--device', default='/dev/video2')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--max-fps', type=int, default=15)
    parser.add_argument('--quality', type=int, default=60, help='JPEG quality 1-100 (default: 60)')
    parser.add_argument('--native', action='store_true',
                        help='Passthrough mode: skip re-encode, send raw camera MJPG directly')
    args = parser.parse_args()

    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    cs = CameraStream(args.device, args.width, args.height, args.max_fps, fourcc,
                      native=args.native, jpeg_quality=args.quality)
    MJPEGHandler.camera_stream = cs
    cs.start()

    server = ThreadedHTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    mode = "native passthrough" if args.native else f"re-encode quality={args.quality}"
    print(f"[camera] Listening on http://0.0.0.0:{args.port}")
    print(f"[camera] Stream: {args.width}x{args.height} @ <= {args.max_fps} fps ({mode})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[camera] Shutting down...")
    finally:
        cs.stop()
        server.shutdown()


if __name__ == '__main__':
    main()
