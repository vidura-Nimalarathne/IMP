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
WHEEL_BASE_M = 0.235

LEFT_TICKS_PER_REV = 766.0
RIGHT_TICKS_PER_REV = 1455.0

# Flip sign later if one side turns out reversed
LEFT_SIGN = 1.0
RIGHT_SIGN = 1.0

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# ===== ODOM FILTER / SANITY LIMITS =====
# Ignore unrealistically fast packet intervals
MIN_DT = 0.02        # 20 ms
# Cap long delays so computed twist does not become meaningless
MAX_DT = 0.30        # 300 ms

# Reject impossible encoder jumps in one update
MAX_TICK_JUMP_L = 80
MAX_TICK_JUMP_R = 80

# Reject impossible robot motion in one update
MAX_LINEAR_STEP_M = 0.08      # 8 cm per update
MAX_ANGULAR_STEP_RAD = 0.8   # ~20 degrees per update

# Optional twist clamps for published /odom
MAX_LINEAR_VEL = 0.8          # m/s
MAX_ANGULAR_VEL = 3.0         # rad/s

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
                conn.sendall((message + "\n").encode("ascii", "ignore"))
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


def clamp(val, lo, hi):
    if val < lo:
        return lo
    if val > hi:
        return hi
    return val


def send_uart(ser, msg):
    with ser_lock:
        ser.write((msg + "\n").encode("ascii", "ignore"))


def parse_odo_packet(line):
    """
    Accept only exact packets like:
      <ODO,596,628>
    Return (left, right) or None
    """
    if not line:
        return None

    if not (line.startswith("<ODO,") and line.endswith(">")):
        return None

    body = line[1:-1]   # remove < >
    parts = body.split(",")

    if len(parts) != 3:
        return None

    if parts[0] != "ODO":
        return None

    try:
        left = int(parts[1])
        right = int(parts[2])
    except ValueError:
        return None

    return (left, right)


def publish_odom(odom_pub, tf_broadcaster, left_ticks, right_ticks):
    global latest_left, latest_right
    global pose_x, pose_y, pose_th
    global prev_left, prev_right, prev_time

    now = rospy.Time.now()

    with odom_lock:
        latest_left = left_ticks
        latest_right = right_ticks

        # First valid sample: initialize baseline only
        if prev_left is None:
            prev_left = left_ticks
            prev_right = right_ticks
            prev_time = now
            return

        raw_dt = (now - prev_time).to_sec()

        # Ignore unrealistically tiny dt bursts
        if raw_dt < MIN_DT:
            return

        # Clamp very long dt gaps
        dt = raw_dt
        if dt > MAX_DT:
            dt = MAX_DT

        dleft_ticks = (left_ticks - prev_left) * LEFT_SIGN
        dright_ticks = (right_ticks - prev_right) * RIGHT_SIGN

        # Move baseline forward immediately after reading deltas
        prev_left = left_ticks
        prev_right = right_ticks
        prev_time = now

    # Reject impossible tick jumps
    
        turning = (dleft_ticks * dright_ticks) < 0

        if turning:
            limit_l = 140
            limit_r = 140
        else:
            limit_l = 80
            limit_r = 80
    
        orig_dl = dleft_ticks
        orig_dr = dright_ticks
    
        dleft_ticks = clamp(dleft_ticks, -limit_l, limit_l)
        dright_ticks = clamp(dright_ticks, -limit_r, limit_r)
    
        if dleft_ticks != orig_dl or dright_ticks != orig_dr:
            print("[ODOM] Clamped tick jump: dL={}→{}, dR={}→{}".format(
                int(orig_dl), int(dleft_ticks), int(orig_dr), int(dright_ticks)
            ))

    left_dist_per_tick = (math.pi * WHEEL_DIAMETER_M) / LEFT_TICKS_PER_REV
    right_dist_per_tick = (math.pi * WHEEL_DIAMETER_M) / RIGHT_TICKS_PER_REV

    dl = dleft_ticks * left_dist_per_tick
    dr = dright_ticks * right_dist_per_tick

    dc = (dl + dr) / 2.0
    dth = (dr - dl) / WHEEL_BASE_M

    # Reject impossible motion steps
    if abs(dc) > MAX_LINEAR_STEP_M or abs(dth) > MAX_ANGULAR_STEP_RAD:
        print("[ODOM] Rejected motion step: dc={:.4f} m, dth={:.4f} rad".format(dc, dth))
        return

    # Midpoint integration
    pose_x += dc * math.cos(pose_th + dth / 2.0)
    pose_y += dc * math.sin(pose_th + dth / 2.0)
    pose_th = normalize_angle(pose_th + dth)

    vx = dc / dt
    vth = dth / dt

    # Clamp published twist to avoid absurd spikes in /odom
    vx = clamp(vx, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
    vth = clamp(vth, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

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
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = vth

    odom_pub.publish(odom)


def serial_reader(ser, odom_pub, tf_broadcaster):
    print("[UART] Reader started")

    while not rospy.is_shutdown():
        try:
            raw = ser.readline()

            if not raw:
                continue

            # Decode safely. Any corrupted bytes are ignored.
            line = raw.decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            parsed = parse_odo_packet(line)
            if parsed is None:
                # Ignore bad/corrupted packets silently
                continue

            left, right = parsed

            # Print only sanitized packet
            print("[MEGA] <ODO,{},{:d}>".format(left, right))

            publish_odom(odom_pub, tf_broadcaster, left, right)

            # Rebroadcast only clean data
            broadcast_to_clients("<ODO,{},{:d}>".format(left, right))

        except Exception as e:
            print("[UART ERROR]", e)
            time.sleep(0.05)


def handle_client(conn, addr, ser):
    print("[NET] Client connected:", addr)

    with clients_lock:
        clients.append(conn)

    try:
        while not rospy.is_shutdown():
            data = conn.recv(1024)
            if not data:
                break

            msg = data.decode("ascii", "ignore").strip()
            if not msg:
                continue

            print("[NET RX]", msg)

            if msg.startswith("<CMD:") and len(msg) >= 7:
                cmd = msg[5]

                # Keep old UI compatibility
                if cmd == "L":
                    cmd = "D"

                if cmd in VALID:
                    send_uart(ser, "<{}>".format(cmd))
                    conn.sendall("<ACK:{}>\n".format(cmd).encode("ascii", "ignore"))
                else:
                    conn.sendall("<ERR>\n".encode("ascii", "ignore"))

    except Exception as e:
        print("[NET ERROR]", e)

    finally:
        try:
            send_uart(ser, "<S>")
        except Exception:
            pass

        with clients_lock:
            try:
                clients.remove(conn)
            except ValueError:
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
        try:
            conn, addr = s.accept()
        except Exception as e:
            print("[NET ACCEPT ERROR]", e)
            continue

        t2 = threading.Thread(
            target=handle_client,
            args=(conn, addr, ser)
        )
        t2.daemon = True
        t2.start()


if __name__ == "__main__":
    main()
