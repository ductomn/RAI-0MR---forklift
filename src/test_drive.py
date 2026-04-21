import asyncio
import websockets

async def control_forklift():
    # Zadejte IP adresu vašeho ESP32. Pokud je ESP32 jako Access Point, bývá to 192.168.4.1
    uri = "ws://192.168.4.1/CarInput"

    print("Připojuji se k vozíku...")
    try:
        async with websockets.connect(uri) as websocket:
            print("Připojeno! Začínám sekvenci...")

            # 1. Rozsvítit světla
            await websocket.send("light,1")
            
            # 2. Jízda vpřed po dobu 2 sekund
            print("Jedeme vpřed...")
            #await websocket.send("throttle,-200")
            await asyncio.sleep(2)
            await websocket.send("throttle,0")
            await asyncio.sleep(0.5)
            # 4. Zvednutí vidlí
            #mast: 5-nahoru, 6-dolu
            #steering: 90-rovne, 
            print("Zvedám vidle...")
            await websocket.send("steering,100")
            await asyncio.sleep(2)

            await websocket.send("steering,100")
            await websocket.send("throttle,-200")
            await asyncio.sleep(2)
            await websocket.send("throttle,0")
            await asyncio.sleep(2)
            await websocket.send("steering,90")
            await asyncio.sleep(1)

            print("Sekvence dokončena.")
            
    except Exception as e:
        print(f"Chyba připojení: {e}")

# Spuštění programu
asyncio.run(control_forklift())