import sys
import os

from PySide6.QtWidgets import QApplication

from gif_player import GifPlayer
from fifo_handler import FifoHandler


def main():
    app = QApplication(sys.argv)

    gif_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gif_source")

    player = GifPlayer(gif_dir, initial_gif="standby",
                       initial_loops=-1, initial_interval=2.0)

    handler = FifoHandler()
    handler.play_requested.connect(player.play)
    handler.stop_requested.connect(player.stop)
    handler.resume_requested.connect(player.resume)
    handler.interval_changed.connect(player.set_interval)
    handler.quit_requested.connect(app.quit)

    code = app.exec()
    handler.cleanup()
    sys.exit(code)


if __name__ == "__main__":
    main()
