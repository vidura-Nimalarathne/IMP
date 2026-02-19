import serial, time
ser = serial.serial('/dev/ttyTHS1' , 9600, timeout=1)
time.sleep(1)
print("UART opened successfully")

while True:
    ser.write(b"Hello from jetson\n")
  print("sent to Mega")

raw = ser.readline()
print("RAW:", raw)
time.sleep(1)
