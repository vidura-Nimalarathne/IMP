import serial
import time

ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
time.sleep(1)

def send(cmd):
    packet = f"<{cmd}>\n"
    ser.write(packet.encode())
    print("Sent:", packet.strip())

    reply = ser.readline().decode(errors="ignore").strip()
    if reply:
        print("Reply:", reply)

commands = ['F', 'R', 'C', 'S']

for c in commands:
    send(c)
    time.sleep(2)

print("Done.")
