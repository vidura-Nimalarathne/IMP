#!/usr/bin/env python
from __future__ import print_function

import math
import socket
import threading
import time
import serial

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

UART_DEV = "/dev/ttyTHS1"
UART_BAUD = 9600

HOST = "0.0.0.0"
PORT = 5005

VALID = {"F", "R", "S", "C", "A", "D", "L"}

# ===== ROBOT PARAMETERS =====
WHEEL_DIAMETER_M = 0.065
WHEEL_BASE_M     = 0.190
TICKS_PER_REV    = 390.0

LEFT_SIGN  = 1.0
RIGHT_SIGN = 1.0

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

clients_lock = threading.Lock()
clients = []

ser_lock = threading.Lock()

odom_lock = threading.Lock()
latest_left = 0
latest_right = 0

pose_x = 0.0
pose_y = 0.0
pose_th = 0.0
prev_left = None
prev_right = None
prev_time = None


def broadcast_to_clients(message):
    dead = []
    with clients_lock:
        for conn in clients:
            try:
                conn.sendall((message + "\n").encode("utf-8"))
            except Exception:
                dead.append(conn)

        for conn in dead:
            try:
                clients.remove(conn)
            except ValueError:
                pass


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def send_uart(ser, msg):
    with ser_lock:
        ser.write((msg + "\n").encode("utf-8"))


def publish_odom(odom_pub, tf_broadcaster, left_ticks, right_ticks):
    global latest_left, latest_right
    global pose_x, pose_y, pose_th
    global prev_left, prev_right, prev_time

    now = rospy.Time.now()

    with odom_lock:
        latest_left = left_ticks
        latest_right = right_ticks

        if prev_left is None:
            prev_left = left_ticks
            prev_right = right_ticks
            prev_time = now
            return

        dt = (now - prev_time).to_sec()
        if dt <= 0.0:
            dt = 1e-3

        dleft_ticks  = (left_ticks  - prev_left)  * LEFT_SIGN
        dright_ticks = (right_ticks - prev_right) * RIGHT_SIGN

        prev_left = left_ticks
        prev_right = right_ticks
        prev_time = now

    dist_per_tick = (math.pi * WHEEL_DIAMETER_M) / TICKS_PER_REV

    dl = dleft_ticks * dist_per_tick
    dr = dright_ticks * dist_per_tick

    dc = (dl + dr) / 2.0
    dth = (dr - dl) / WHEEL_BASE_M

    # ===== TEMP TEST CHANGE START =====
    # Real odometry update is temporarily bypassed.
    # This forces fake forward motion so we can test whether:
    # 1) gmapping accepts odom
    # 2) /map updates
    # 3) UI stops showing "Waiting for /map ..."
    pose_x += 0.01
    pose_y += 0.0
    pose_th = normalize_angle(pose_th + dth)
    # ===== TEMP TEST CHANGE END =====

    vx = dc / dt
    vth = dth / dt

    q = tf.transformations.quaternion_from_euler(0, 0, pose_th)

    tf_broadcaster.sendTransform(
        (pose_x, pose_y, 0.0),
        q,
        now,
        BASE_FRAME,
        ODOM_FRAME
    )

    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = ODOM_FRAME
    odom.child_frame_id = BASE_FRAME

    odom.pose.pose.position.x = pose_x
    odom.pose.pose.position.y = pose_y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = Quaternion(*q)

    odom.twist.twist.linear.x = vx
    odom.twist.twist.angular.z = vth

    odom_pub.publish(odom)


def serial_reader(ser, odom_pub, tf_broadcaster):
    print("[UART] Reader started")

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print("[MEGA] {}".format(line))

            if line.startswith("<ODO,") and line.endswith(">"):
                body = line[1:-1]
                parts = body.split(",")

                if len(parts) == 3:
                    left = int(parts[1])
                    right = int(parts[2])

                    publish_odom(odom_pub, tf_broadcaster, left, right)
                    broadcast_to_clients("<ODO,{},{:d}>".format(left, right))

        except Exception as e:
            print("[UART ERROR]", e)
            time.sleep(0.1)


def handle_client(conn, addr, ser):
    print("[NET] Client connected:", addr)

    with clients_lock:
        clients.append(conn)

    try:
        while not rospy.is_shutdown():
            data = conn.recv(1024)
            if not data:
                break

            msg = data.decode().strip()
            print("[NET RX]", msg)

            if msg.startswith("<CMD:"):
                cmd = msg[5]
                if cmd == "L":
                    cmd = "D"

                if cmd in VALID:
                    send_uart(ser, "<{}>".format(cmd))
                    conn.sendall("<ACK:{}>\n".format(cmd).encode())
                else:
                    conn.sendall("<ERR>\n".encode())

    except Exception as e:
        print("[NET ERROR]", e)

    finally:
        try:
            send_uart(ser, "<S>")
        except Exception:
            pass
        conn.close()
        print("[NET] Client disconnected:", addr)


def main():
    rospy.init_node("jcserver_mapping_ros")

    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=20)
    tf_broadcaster = tf.TransformBroadcaster()

    print("[SYS] Starting jcserver_mapping_ros...")

    ser = serial.Serial(UART_DEV, UART_BAUD, timeout=0.05)
    time.sleep(0.5)
    print("[UART] Opened", UART_DEV)

    t1 = threading.Thread(
        target=serial_reader,
        args=(ser, odom_pub, tf_broadcaster)
    )
    t1.daemon = True
    t1.start()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)

    print("[NET] Listening on", HOST, PORT)

    while not rospy.is_shutdown():
        conn, addr = s.accept()

        t2 = threading.Thread(
            target=handle_client,
            args=(conn, addr, ser)
        )
        t2.daemon = True
        t2.start()


if __name__ == "__main__":
    main()
