import websocket  
from typing import Optional

class WebsocketInterface:
    def __init__(self, uri: str):
        self.uri = uri
        self.websocket: Optional[websocket.WebSocket] = None

        try:
            self.open()
        except Exception as e:
            print(f"Failed to connect to websocket at {uri}: {e}")
            self.websocket = None

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

        self.safe_send(message)

    def safe_send(self, message: str) -> None:
        """Send message with error handling and automatic reconnect."""
        if self.websocket is None:
            raise RuntimeError("Websocket not open. Call open() first.")

        try:
            self.websocket.send(message)
        except Exception as e:
            print(f"Error sending message: {e}")
            print("Attempting to reconnect...")
            try:
                self.close()
                self.open()
                self.websocket.send(message)
                print("Message sent after reconnect.")
            except Exception as reconnect_error:
                print(f"Reconnect failed: {reconnect_error}")
                raise RuntimeError("Failed to send message after reconnecting.") from reconnect_error

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
    
    def mastControl_stop(self) -> None:
        self.send("mast,0")

    def mastControl(self, value: int) -> None:
        """Value should be 5 for up and 6 for down."""
        if value == 5:
            self.mastControl_up()
        elif value == 6:
            self.mastControl_down()
        elif value == 0:
            self.mastControl_stop()
        else:
            raise ValueError("Invalid mast control value. Use 5 for up and 6 for down.")

    def mastTilt_forward(self) -> None:
        self.send("mTilt,1")

    def mastTilt_backward(self) -> None:
        self.send("mTilt,2")