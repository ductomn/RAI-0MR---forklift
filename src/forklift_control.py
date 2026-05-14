import websocket  
from typing import Optional

class WebsocketInterface:
    def __init__(self, uri: str):
        self.uri = uri
        self.websocket: Optional[websocket.WebSocket] = None

    def open(self) -> None:
        """Open websocket connection if it is not already open."""
        if self.websocket is not None:
            print("Websocket already open.")
            return

        self.websocket = websocket.create_connection(self.uri)
        print("Websocket opened.")

    def close(self) -> None:
        """Close websocket connection if it is open."""
        if self.websocket is None:
            print("Websocket already closed.")
            return

        self.websocket.close()
        self.websocket = None
        print("Websocket closed.")

    def send(self, message: str) -> None:
        """Send raw message over an already open connection."""
        if self.websocket is None:
            raise RuntimeError("Websocket not open. Call open() first.")

        self.websocket.send(message)

    def __enter__(self):
        """Context manager support (synchronous)"""
        self.open()
        return self
        
    def __exit__(self, exc_type, exc, tb):
        self.close()


class ForkliftClient(WebsocketInterface):
    def send_throttle(self, value: int) -> None:
        """Value should be between -255 and 255, where negative is forward."""
        self.send(f"throttle,{value}")

    def stop_throttle(self) -> None:
        self.send_throttle(0)

    def send_steering(self, value: int) -> None:
        """Value should be between 0 and 180, where 80-100 is straight. Under 80 is right, over 100 is left."""
        self.send(f"steering,{value}")

    def stop_steering(self) -> None:
        self.send_steering(90)

    def mastControl_up(self) -> None:
        self.send("mast,5")

    def mastControl_down(self) -> None:
        self.send("mast,6")

    def mastTilt_forward(self) -> None:
        self.send("mTilt,1")

    def mastTilt_backward(self) -> None:
        self.send("mTilt,2")