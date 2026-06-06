import os
from enum import Enum, auto

from PySide6.QtWidgets import QMainWindow, QLabel
from PySide6.QtCore import Qt, QTimer, QEvent
from PySide6.QtGui import QMovie


class State(Enum):
    PLAYING = auto()
    WAITING = auto()
    PAUSED = auto()


class GifPlayer(QMainWindow):
    def __init__(self, gif_dir: str, initial_gif: str = "standby",
                 initial_loops: int = -1, initial_interval: float = 2.0):
        super().__init__()

        self._gif_dir = gif_dir
        self._current_gif = initial_gif
        self._loops_remaining = initial_loops
        self._interval_ms = int(initial_interval * 1000)
        self._state = State.PAUSED
        self._prev_frame = -1

        self._file_path = self._build_path(initial_gif)

        self.setWindowFlags(Qt.FramelessWindowHint)
        self.showFullScreen()

        self.label = QLabel(self)
        self.label.setScaledContents(True)
        self.label.installEventFilter(self)
        self.setCentralWidget(self.label)
        self.centralWidget().setContentsMargins(0, 0, 0, 0)

        self.movie = QMovie(self._file_path)
        self.movie.setCacheMode(QMovie.CacheAll)
        self.movie.frameChanged.connect(self._on_frame_changed)
        self.label.setMovie(self.movie)

        self._interval_timer = QTimer(self)
        self._interval_timer.setSingleShot(True)
        self._interval_timer.timeout.connect(self._on_interval_done)

        self._long_press_timer = QTimer(self)
        self._long_press_timer.setSingleShot(True)
        self._long_press_timer.timeout.connect(self.close)

        self.movie.start()
        self._state = State.PLAYING

    def _build_path(self, name: str) -> str:
        return os.path.join(self._gif_dir, f"{name}.gif")

    def _on_frame_changed(self, frame: int):
        if self._state != State.PLAYING:
            return

        if self._prev_frame > 0 and frame <= self._prev_frame:
            self._on_loop_completed()

        self._prev_frame = frame

    def _on_loop_completed(self):
        if self._loops_remaining > 0:
            self._loops_remaining -= 1
            if self._loops_remaining == 0:
                self._state = State.PAUSED
                self.movie.jumpToFrame(self._prev_frame)
                self.movie.setPaused(True)
                return

        self.movie.setPaused(True)
        self._state = State.WAITING
        self._interval_timer.start(self._interval_ms)

    def _on_interval_done(self):
        if self._state != State.WAITING:
            return

        self._prev_frame = -1
        self._state = State.PLAYING
        self.movie.setPaused(False)

    def play(self, name: str, loops: int = 0, interval: float = -1.0):
        path = self._build_path(name)
        if not os.path.isfile(path):
            return

        if interval >= 0:
            self._interval_ms = int(interval * 1000)

        self._interval_timer.stop()
        self._current_gif = name
        self._loops_remaining = -1 if loops == 0 else loops
        self._prev_frame = -1

        self.movie.stop()
        self.movie.setFileName(path)
        self.movie.start()
        self._state = State.PLAYING

    def stop(self):
        self._interval_timer.stop()
        self.movie.setPaused(True)
        self._state = State.PAUSED

    def resume(self):
        if self._state != State.PAUSED:
            return

        if self.movie.state() == QMovie.Paused:
            self.movie.setPaused(False)
            self._state = State.PLAYING
        else:
            self._prev_frame = -1
        self.movie.start()
        self._state = State.PLAYING

    def set_interval(self, seconds: float):
        self._interval_ms = int(seconds * 1000)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.MouseButtonPress:
            self._long_press_timer.start(2000)
        elif event.type() == QEvent.MouseButtonRelease:
            self._long_press_timer.stop()
        return super().eventFilter(obj, event)
