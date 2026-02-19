import socket
import threading
import serial
import time

UART_DEV = "/dev/ttyTHS1"
UART_BAUD = 9600

HOST = "0.0.0.0"
PORT = 5005

VALID = {"F", "R","S", "L", "D"}

def handle_client(conn, addr, ser):
    print(f"[NET] Client connected: {addr}")
    conn.settimeout(1.0)
    buf = b""

    try:
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break
                buf += data

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    msg = line.decode(errors="ignore").strip()

                    # Expect: <CMD:X>
                    if msg.startswith("<CMD:") and msg.endswith(">") and len(msg) == 7:
                        cmd = msg[5]
                        if cmd in VALID:
                            ser.write(f"<{cmd}>\n".encode())
                            conn.sendall(f"<ACK:{cmd}>\n".encode())
                        else:
                            conn.sendall(b"<ERR:bad_cmd>\n")
                    else:
                        conn.sendall(b"<ERR:bad_format>\n")

            except socket.timeout:
                continue
    finally:
        # safety stop on disconnect
        try:
            ser.write(b"<S>\n")
        except:
            pass
        conn.close()
        print(f"[NET] Client disconnected: {addr}")

def main():
    ser = serial.Serial(UART_DEV, UART_BAUD, timeout=0.2)
    time.sleep(0.5)
    print("[UART] Opened")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"[NET] Listening on {HOST}:{PORT}")

    while True:
        conn, addr = s.accept()
        threading.Thread(target=handle_client, args=(conn, addr, ser), daemon=True).start()

if __name__ == "__main__":
    main()
