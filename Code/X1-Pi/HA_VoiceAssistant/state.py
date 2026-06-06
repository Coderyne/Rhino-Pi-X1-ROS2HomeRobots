"""线程安全的状态机模块"""

import enum
import threading
from typing import Callable


class State(enum.Enum):
    """流水线状态枚举"""

    IDLE = "idle"            # 空闲：等待唤醒词
    LISTENING = "listening"  # 录音中：采集用户语音
    PROCESSING = "processing"  # 处理中：等待 LLM 响应
    SPEAKING = "speaking"    # 播报中：TTS 播放语音


class StateMachine:
    """线程安全的状态机，支持状态转换回调

    Args:
        initial_state: 初始状态
    """

    def __init__(self, initial_state: State = State.IDLE) -> None:
        self._state = initial_state
        self._lock = threading.Lock()
        self._callbacks: dict[State, list[Callable[[State, State], None]]] = {
            s: [] for s in State
        }

    @property
    def current(self) -> State:
        """获取当前状态（线程安全）"""
        with self._lock:
            return self._state

    def transition(self, new_state: State) -> bool:
        """尝试原子状态转换

        Returns:
            True 表示转换成功，False 表示状态未变（已在目标状态）
        """
        with self._lock:
            if self._state == new_state:
                return False
            old_state = self._state
            self._state = new_state

        # 在锁外触发回调，避免死锁
        for cb in self._callbacks.get(old_state, []):
            cb(old_state, new_state)
        for cb in self._callbacks.get(new_state, []):
            cb(old_state, new_state)
        return True

    def on_enter(self, state: State, callback: Callable[[State, State], None]) -> None:
        """注册进入某状态时的回调"""
        self._callbacks[state].append(callback)

    def is_idle(self) -> bool:
        return self.current == State.IDLE

    def is_listening(self) -> bool:
        return self.current == State.LISTENING

    def is_processing(self) -> bool:
        return self.current == State.PROCESSING

    def is_speaking(self) -> bool:
        return self.current == State.SPEAKING
