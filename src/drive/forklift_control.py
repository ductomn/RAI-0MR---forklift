import threading
import websocket
from typing import Optional


class WebsocketInterface:
    def __init__(self, uri: Optional[str] = None, auto_connect: bool = False):
        self.uri = uri
        self.websocket: Optional[websocket.WebSocket] = None
        self._lock = threading.RLock()

        if auto_connect and uri:
            self.open()

    @property
    def is_connected(self) -> bool:
        return self.websocket is not None and bool(self.websocket.connected)

    def open(self, uri: Optional[str] = None) -> None:
        """Open the connection, optionally replacing the current URI."""
        with self._lock:
            if uri:
                self.uri = uri
            if not self.uri:
                raise ValueError("A WebSocket URI is required.")
            if self.is_connected:
                return

            self.close()
            self.websocket = websocket.create_connection(self.uri, timeout=3)
            print(f"Websocket opened at {self.uri}.")

    def close(self) -> None:
        with self._lock:
            socket = self.websocket
            self.websocket = None
            if socket is not None:
                try:
                    socket.close()
                except Exception as error:
                    print(f"Error closing websocket: {error}")

    def send(self, message: str) -> None:
        """Send a message and attempt one reconnect if the connection drops."""
        with self._lock:
            if not self.is_connected:
                raise RuntimeError("Websocket is not connected.")

            try:
                self.websocket.send(message)
            except Exception as error:
                print(f"Error sending message: {error}; attempting reconnect.")
                self.close()
                try:
                    self.open()
                    self.websocket.send(message)
                except Exception as reconnect_error:
                    self.close()
                    raise RuntimeError(
                        "Failed to send message after reconnecting."
                    ) from reconnect_error

    # Kept for compatibility with existing callers.
    safe_send = send

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


class ForkliftClient(WebsocketInterface):
    def send_throttle(self, value: int) -> None:
        """Value should be -255..255, where negative is forward."""
        self.send(f"throttle,{max(-255, min(255, value))}")

    def stop_throttle(self) -> None:
        self.send_throttle(0)

    def send_steering(self, value: int) -> None:
        """Value should be 0..180, where approximately 90 is straight."""
        self.send(f"steering,{max(0, min(180, value))}")

    def stop_steering(self) -> None:
        self.send_steering(90)

    def mastControl_up(self) -> None:
        self.send("mast,5")

    def mastControl_down(self) -> None:
        self.send("mast,6")

    def mastControl_stop(self) -> None:
        self.send("mast,0")

    def mastControl(self, value: int) -> None:
        if value == 5:
            self.mastControl_up()
        elif value == 6:
            self.mastControl_down()
        elif value == 0:
            self.mastControl_stop()
        else:
            raise ValueError("Invalid mast value. Use 5 for up, 6 for down, or 0.")

    def mastTilt_forward(self) -> None:
        self.send("mTilt,1")

    def mastTilt_backward(self) -> None:
        self.send("mTilt,2")
