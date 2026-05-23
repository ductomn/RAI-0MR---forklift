import time

from forklift_control import ForkliftClient


def test_forklift_sequence(websocket: ForkliftClient) -> None:
    """Run the standard forklift control sequence over an open websocket."""
    print("Connected...")

    websocket.stop_steering()
    time.sleep(2)

    # print("Moving forward...")
    # websocket.send("throttle,-200")
    # time.sleep(1.5)
    # websocket.stop_throttle()
    # time.sleep(0.5)

    # print("Moving back...")
    # websocket.send("throttle,200")
    # time.sleep(1.5)
    # websocket.stop_throttle()
    # time.sleep(0.5)

    # print("Turning left...")
    # websocket.send_steering(110)
    # time.sleep(2)
    # websocket.send("throttle,-200")
    # time.sleep(1.5)
    # websocket.stop_throttle()
    # time.sleep(0.5)

    # websocket.stop_steering()
    # time.sleep(2)

    # print("Turning right...")
    # websocket.send_steering(70)
    # time.sleep(2)
    # websocket.send("throttle,-200")
    # time.sleep(1.5)
    # websocket.stop_throttle()
    # time.sleep(0.5)

    # websocket.stop_steering()
    # time.sleep(2)

    print("Turning left...")
    websocket.send_steering(120)
    time.sleep(2)

    print("Returning to center...")
    websocket.send_steering(90)
    time.sleep(2)

    print("Turning right...")
    websocket.send_steering(60)
    time.sleep(2)

    print("Returning to center...")
    websocket.send_steering(100)
    time.sleep(2)

    print("Sequence completed.")


def control_forklift():
    # IP Address: 192.168.4.1
    uri = "ws://192.168.4.1/CarInput"

    print("Connecting to forklift...")
    websocket = ForkliftClient(uri)
    try:
        websocket.open()
        test_forklift_sequence(websocket)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        websocket.close()

if __name__ == "__main__":
    control_forklift()
