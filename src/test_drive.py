import asyncio

from forklift_control import ForkliftClient


async def test_forklift_sequence(websocket: ForkliftClient) -> None:
    """Run the standard forklift control sequence over an open websocket."""
    print("Connected...")

    await websocket.stop_steering()
    await asyncio.sleep(2)

    print("Moving forward...")
    await websocket.send("throttle,-200")
    await asyncio.sleep(1.5)
    await websocket.stop_throttle()
    await asyncio.sleep(0.5)

    print("Moving back...")
    await websocket.send("throttle,200")
    await asyncio.sleep(1.5)
    await websocket.stop_throttle()
    await asyncio.sleep(0.5)

    print("Turning left...")
    await websocket.send_steering(110)
    await asyncio.sleep(2)
    await websocket.send("throttle,-200")
    await asyncio.sleep(1.5)
    await websocket.stop_throttle()
    await asyncio.sleep(0.5)

    await websocket.stop_steering()
    await asyncio.sleep(2)

    print("Turning right...")
    await websocket.send_steering(70)
    await asyncio.sleep(2)
    await websocket.send("throttle,-200")
    await asyncio.sleep(1.5)
    await websocket.stop_throttle()
    await asyncio.sleep(0.5)

    await websocket.stop_steering()
    await asyncio.sleep(2)

    print("Sequence completed.")


async def control_forklift():
    # IP Address: 192.168.4.1
    uri = "ws://192.168.4.1/CarInput"

    print("Connecting to forklift...")
    websocket = ForkliftClient(uri)
    try:
        await websocket.open()
        await test_forklift_sequence(websocket)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        await websocket.close()

# Run the control sequence
asyncio.run(control_forklift())
