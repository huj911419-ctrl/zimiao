import serial
import threading
import time

ports = ['COM3', 'COM4', 'COM7', 'COM9']
baud = 115200

def read_port(port):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[{port}] Opened")
        start = time.time()
        while time.time() - start < 5:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[{port}] {line}")
        ser.close()
    except Exception as e:
        print(f"[{port}] Error: {e}")

threads = []
for p in ports:
    t = threading.Thread(target=read_port, args=(p,))
    t.start()
    threads.append(t)

for t in threads:
    t.join()

print("Done")
