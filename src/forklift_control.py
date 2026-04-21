import websockets
from typing import Optional


class WebsocketInterface:
    def __init__(self, uri: str):
        self.uri = uri
        self.websocket: Optional[websockets.ClientConnection] = None

    async def _connect(self) -> websockets.ClientConnection:
        """Create a websocket connection for the configured URI."""
        return await websockets.connect(self.uri)

    async def open(self) -> None:
        """Open websocket connection if it is not already open."""
        if self.websocket is not None:
            print("Websocket already open.")
            return

        self.websocket = await self._connect()
        print("Websocket opened.")

    async def close(self) -> None:
        """Close websocket connection if it is open."""
        if self.websocket is None:
            print("Websocket already closed.")
            return

        await self.websocket.close()
        self.websocket = None
        print("Websocket closed.")

    async def send(self, message: str) -> None:
        """Send raw message over an already open connection."""
        if self.websocket is None:
            raise RuntimeError(
                "Websocket not open. Call open() first.")

        await self.websocket.send(message)

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()


class ForkliftClient(WebsocketInterface):
    async def send_throttle(self, value: int) -> None:
        """Value should be between -255 and 255, where negative is forward."""
        await self.send(f"throttle,{value}")

    async def stop_throttle(self) -> None:
        await self.send_throttle(0)

    async def send_steering(self, value: int) -> None:
        """Value should be between 0 and 180, where 80-100 is straight. Under 80 is right, over 100 is left."""
        await self.send(f"steering,{value}")

    async def stop_steering(self) -> None:
        await self.send_steering(90)

    async def mastControl_up(self) -> None:
        await self.send(f"mast,5")

    async def mastControl_down(self) -> None:
        await self.send(f"mast,6")

    async def mastTilt_forward(self) -> None:
        await self.send(f"mTilt,1")

    async def mastTilt_backward(self) -> None:
        await self.send(f"mTilt,2")
