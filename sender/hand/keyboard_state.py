"""Thread-safe keyboard state tracker shared by hand senders.

Used by both manus_sender.py and realsense_sender.py to capture
button events (e-stop, reset, quit, speed up/down) for inclusion
in UDP packets to the receiver.

Key mappings:
    Space   = E-Stop
    R       = Reset
    Q/Esc   = Quit
    +/=     = Speed up
    -       = Speed down
"""

import threading

try:
    from pynput import keyboard
except ImportError:
    keyboard = None


class KeyboardState:
    """Thread-safe keyboard state tracker using pynput."""

    def __init__(self):
        self._lock = threading.Lock()
        self._estop = False
        self._reset = False
        self._quit = False
        self._speed_up = False
        self._speed_down = False
        self._listener = None

    def start(self):
        if keyboard is None:
            raise ImportError(
                "pynput not installed. Run: pip install pynput"
            )
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.daemon = True
        self._listener.start()
        print("[Keyboard] Listener started")
        print("[Keyboard] Space=E-Stop, R=Reset, Q/Esc=Quit, +/-=Speed")

    def stop(self):
        if self._listener is not None:
            self._listener.stop()
            self._listener = None

    def get_and_clear(self) -> dict:
        """Get current button state and clear edge-triggered flags."""
        with self._lock:
            state = {
                "estop": self._estop,
                "reset": self._reset,
                "quit": self._quit,
                "speed_up": self._speed_up,
                "speed_down": self._speed_down,
            }
            self._estop = False
            self._reset = False
            self._quit = False
            self._speed_up = False
            self._speed_down = False
        return state

    def _on_press(self, key):
        with self._lock:
            try:
                if key == keyboard.Key.space:
                    self._estop = True
                elif key == keyboard.Key.esc:
                    self._quit = True
                elif hasattr(key, "char") and key.char is not None:
                    ch = key.char.lower()
                    if ch == "r":
                        self._reset = True
                    elif ch == "q":
                        self._quit = True
                    elif ch in ("+", "="):
                        self._speed_up = True
                    elif ch == "-":
                        self._speed_down = True
            except AttributeError:
                pass

    def _on_release(self, key):
        pass
