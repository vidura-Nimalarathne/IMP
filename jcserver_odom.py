#!/usr/bin/env python3

import socket
import threading
import serial
import time

UART_DEV = "/dev/ttyTHS1"
UART_BAUD = 9600

HOST = "0.0.0.0"
PORT = 5005

# Commands accepted from network client
VALID = {"F", "R", "S", "C", "A", "D"}

# Shared state
clients_lock = threading.Lock()
clients = []

odom_lock = threading.Lock()
latest_left = 0
latest_right = 0


def broadcast_to_clients(message: str):
    """Send a message to all connected TCP clients."""
    dead = []

    with clients_lock:
        for conn in clients:
            try:
                conn.sendall((message + "\n").encode())
            except Exception:
                dead.append(conn)

        for conn in dead:
            try:
                clients.remove(conn)
            except ValueError:
                pass


def serial_reader(ser):
    """
    Continuously read UART data coming from Arduino Mega.
    Expected examples:
      <ODO,1356,1271>
      <ACK:F>
      <ERR:OBST>
    """
    global latest_left, latest_right

    print("[UART] Reader started")

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print(f"[MEGA] {line}")

            # Odometry packet: <ODO,left,right>
            if line.startswith("<ODO,") and line.endswith(">"):
                try:
                    body = line[1:-1]          # remove < >
                    parts = body.split(",")    # ["ODO", "1356", "1271"]

                    if len(parts) == 3 and parts[0] == "ODO":
                        left = int(parts[1])
                        right = int(parts[2])

                        with odom_lock:
                            latest_left = left
                            latest_right = right

                        print(f"[ODOM] L={left} R={right}")

                        # Optional: forward odometry to connected client(s)
                        broadcast_to_clients(f"<ODO,{left},{right}>")

                except Exception as e:
                    print(f"[UART] Bad ODO packet: {e}")

            # Forward ACK/ERR from Mega to network clients too
            elif line.startswith("<ACK:") or line.startswith("<ERR:"):
                broadcast_to_clients(line)

        except Exception as e:
            print(f"[UART ERROR] {e}")
            time.sleep(0.2)


def handle_client(conn, addr, ser):
    """
    Handle one TCP client.
    Client sends lines like:
      <CMD:F>
      <CMD:R>
      <CMD:S>
      <GET:ODO>
    """
    print(f"[NET] Client connected: {addr}")
    conn.settimeout(1.0)
    buf = b""

    with clients_lock:
        clients.append(conn)

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

                    if not msg:
                        continue

                    print(f"[NET RX] {msg}")

                    # Teleop command: <CMD:X>
                    if msg.startswith("<CMD:") and msg.endswith(">") and len(msg) == 7:
                        cmd = msg[5]

                        if cmd in VALID:
                            uart_msg = f"<{cmd}>\n"
                            ser.write(uart_msg.encode())
                            print(f"[UART TX] {uart_msg.strip()}")

                            # Immediate ACK from server side
                            conn.sendall(f"<ACK:{cmd}>\n".encode())
                        else:
                            conn.sendall(b"<ERR:bad_cmd>\n")

                    # Odometry request from client
                    elif msg == "<GET:ODO>":
                        with odom_lock:
                            left = latest_left
                            right = latest_right

                        conn.sendall(f"<ODO,{left},{right}>\n".encode())

                    else:
                        conn.sendall(b"<ERR:bad_format>\n")

            except socket.timeout:
                continue

    except Exception as e:
        print(f"[NET ERROR] {addr} -> {e}")

    finally:
        # Safety stop on disconnect
        try:
            ser.write(b"<S>\n")
            print("[UART TX] <S>")
        except Exception:
            pass

        with clients_lock:
            try:
                clients.remove(conn)
            except ValueError:
                pass

        conn.close()
        print(f"[NET] Client disconnected: {addr}")


def main():
    print("[SYS] Starting jcserver_odom...")

    # Open UART to Arduino Mega
    ser = serial.Serial(UART_DEV, UART_BAUD, timeout=0.2)
    time.sleep(0.5)
    print(f"[UART] Opened {UART_DEV} @ {UART_BAUD}")

    # Start UART reader thread
    threading.Thread(target=serial_reader, args=(ser,), daemon=True).start()

    # Start TCP server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)

    print(f"[NET] Listening on {HOST}:{PORT}")

    while True:
        conn, addr = s.accept()
        threading.Thread(target=handle_client, args=(conn, addr, ser), daemon=True).start()


if __name__ == "__main__":
    main()
