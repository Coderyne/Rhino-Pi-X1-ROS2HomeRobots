import os
import shlex
import stat

from PySide6.QtCore import QObject, QSocketNotifier, Signal


FIFO_PATH = "/tmp/gif_cmd"


class FifoHandler(QObject):
    play_requested = Signal(str, int, float)
    stop_requested = Signal()
    resume_requested = Signal()
    interval_changed = Signal(float)
    quit_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self._setup_fifo()
        self._notifier = QSocketNotifier(self._fd, QSocketNotifier.Read, self)
        self._notifier.activated.connect(self._on_readable)
        self._notifier.setEnabled(True)

    def _setup_fifo(self):
        if os.path.exists(FIFO_PATH):
            st = os.stat(FIFO_PATH)
            if not stat.S_ISFIFO(st.st_mode):
                os.remove(FIFO_PATH)
                os.mkfifo(FIFO_PATH)
        else:
            os.mkfifo(FIFO_PATH)

        self._fd = os.open(FIFO_PATH, os.O_RDWR | os.O_NONBLOCK)

    def _on_readable(self):
        try:
            data = os.read(self._fd, 4096)
        except BlockingIOError:
            return

        if not data:
            return

        for line in data.decode(errors="replace").strip().splitlines():
            line = line.strip()
            if not line:
                continue
            self._handle_command(line)

    def _handle_command(self, line: str):
        parts = shlex.split(line)
        if not parts:
            return

        cmd = parts[0].lower()

        if cmd == "play":
            name = parts[1] if len(parts) > 1 else None
            loops = 0
            interval = -1.0
            if len(parts) > 2:
                try:
                    loops = int(parts[2])
                except ValueError:
                    pass
            if len(parts) > 3:
                try:
                    interval = float(parts[3])
                except ValueError:
                    pass
            if name:
                self.play_requested.emit(name, loops, interval)

        elif cmd == "stop":
            self.stop_requested.emit()

        elif cmd == "resume":
            self.resume_requested.emit()

        elif cmd == "interval":
            if len(parts) > 1:
                try:
                    secs = float(parts[1])
                    self.interval_changed.emit(secs)
                except ValueError:
                    pass

        elif cmd == "quit":
            self.quit_requested.emit()

    def cleanup(self):
        self._notifier.setEnabled(False)
        os.close(self._fd)
        try:
            os.unlink(FIFO_PATH)
        except OSError:
            pass
