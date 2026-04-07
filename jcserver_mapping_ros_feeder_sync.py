#!/usr/bin/env python
from __future__ import print_function

import math
import socket
import threading
import time
import serial
import os
import json
from datetime import datetime

import rospy
import tf

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

UART_DEV = "/dev/ttyTHS1"
UART_BAUD = 9600

HOST = "0.0.0.0"
PORT = 5005

CONFIG_PATH = os.path.expanduser("~/feed_config.json")
DEFAULT_FEED_TIME = "11:00"

# Single-character commands that go to Mega
VALID_UART_COMMANDS = {"F", "R", "S", "C", "A", "D", "L", "K"}

# Word commands handled at bridge level
VALID_WORD_COMMANDS = {"AUTO", "FEED_NOW", "GET_STATUS"}

# ===== cmd_vel -> Mega bridge =====
cmd_vel_lock = threading.Lock()
last_cmd_sent = "S"
last_cmd_time = 0.0

CMD_DEADBAND_LINEAR = 0.03
CMD_DEADBAND_ANGULAR = 0.20
CMD_REPEAT_INTERVAL = 0.2

ser_global = None

# ===== autonomous state =====
auto_lock = threading.Lock()
auto_running = False
auto_cancel = False
dog_found = False

# ===== feeder state =====
feed_lock = threading.Lock()
feed_ack_event = threading.Event()
feeding_in_progress = False
feed_time_hhmm = DEFAULT_FEED_TIME
last_feed_date = ""

# Tune this for your robot for roughly 90 deg clockwise
AUTO_TURN_90_TIME = 1.20

# ===== ROBOT PARAMETERS =====
WHEEL_DIAMETER_M = 0.065
WHEEL_BASE_M = 0.235

LEFT_TICKS_PER_REV = 1200
RIGHT_TICKS_PER_REV = 1400

LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# ===== ODOM FILTER / SANITY LIMITS =====
MIN_DT = 0.02
MAX_DT = 0.30

MAX_TICK_JUMP_L_STRAIGHT = 80
MAX_TICK_JUMP_R_STRAIGHT = 80
MAX_TICK_JUMP_L_TURN = 140
MAX_TICK_JUMP_R_TURN = 140

MAX_LINEAR_STEP_M = 0.08
MAX_ANGULAR_STEP_RAD = 0.8

MAX_LINEAR_VEL = 0.8
MAX_ANGULAR_VEL = 3.0

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


def stop_robot():
    global last_cmd_sent, last_cmd_time

    try:
        if ser_global is not None:
            send_uart(ser_global, "<S>")
            with cmd_vel_lock:
                last_cmd_sent = "S"
                last_cmd_time = time.time()
    except Exception as e:
        print("[STOP ERROR]", e)


def load_feed_config():
    global feed_time_hhmm, last_feed_date

    try:
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, "r") as f:
                data = json.load(f)
            feed_time_hhmm = str(data.get("feed_time", DEFAULT_FEED_TIME))
            last_feed_date = str(data.get("last_feed_date", ""))
        else:
            feed_time_hhmm = DEFAULT_FEED_TIME
            last_feed_date = ""
            save_feed_config()
    except Exception as e:
        print("[FEED CONFIG] load error:", e)
        feed_time_hhmm = DEFAULT_FEED_TIME
        last_feed_date = ""


def save_feed_config():
    try:
        data = {
            "feed_time": feed_time_hhmm,
            "last_feed_date": last_feed_date
        }
        with open(CONFIG_PATH, "w") as f:
            json.dump(data, f)
    except Exception as e:
        print("[FEED CONFIG] save error:", e)


def send_status_to_client(conn=None):
    msgs = [
        "TIME:" + datetime.now().strftime("%H:%M:%S"),
        "FEED_TIME:" + feed_time_hhmm,
        "FEEDING:" + ("1" if feeding_in_progress else "0"),
        "LAST_FEED_DATE:" + (last_feed_date or "")
    ]

    if conn is None:
        for m in msgs:
            broadcast_to_clients(m)
    else:
        for m in msgs:
            try:
                conn.sendall((m + "\n").encode("ascii", "ignore"))
            except Exception:
                pass


def dog_detected_callback(msg):
    global dog_found, auto_cancel

    if not msg.data:
        return

    dog_found = True
    auto_cancel = True

    print("[ROS] /dog_detected = True")
    broadcast_to_clients("<EVENT:DOG_DETECTED>")

    stop_robot()


def cmd_vel_callback(msg):
    global last_cmd_sent, last_cmd_time, auto_running

    with auto_lock:
        if auto_running:
            return

    lin = msg.linear.x
    ang = msg.angular.z

    if abs(lin) < CMD_DEADBAND_LINEAR and abs(ang) < CMD_DEADBAND_ANGULAR:
        cmd = "S"
    elif abs(ang) >= CMD_DEADBAND_ANGULAR:
        if ang > 0:
            cmd = "A"
        else:
            cmd = "D"
    elif lin > CMD_DEADBAND_LINEAR:
        cmd = "F"
    elif lin < -CMD_DEADBAND_LINEAR:
        cmd = "R"
    else:
        cmd = "S"

    now = time.time()

    with cmd_vel_lock:
        if cmd != last_cmd_sent or (now - last_cmd_time) >= CMD_REPEAT_INTERVAL:
            try:
                send_uart(ser_global, "<{}>".format(cmd))
                last_cmd_sent = cmd
                last_cmd_time = now
                print("[CMD_VEL] lin={:.3f}, ang={:.3f} -> {}".format(lin, ang, cmd))
            except Exception as e:
                print("[CMD_VEL ERROR]", e)


def parse_odo_packet(line):
    if not line:
        return None

    if not (line.startswith("<ODO,") and line.endswith(">")):
        return None

    body = line[1:-1]
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

        if prev_left is None:
            prev_left = left_ticks
            prev_right = right_ticks
            prev_time = now
            return

        raw_dt = (now - prev_time).to_sec()

        if raw_dt < MIN_DT:
            return

        dt = raw_dt
        if dt > MAX_DT:
            dt = MAX_DT

        dleft_ticks = (left_ticks - prev_left) * LEFT_SIGN
        dright_ticks = (right_ticks - prev_right) * RIGHT_SIGN

        prev_left = left_ticks
        prev_right = right_ticks
        prev_time = now

    turning = (dleft_ticks * dright_ticks) < 0

    if turning:
        limit_l = MAX_TICK_JUMP_L_TURN
        limit_r = MAX_TICK_JUMP_R_TURN
    else:
        limit_l = MAX_TICK_JUMP_L_STRAIGHT
        limit_r = MAX_TICK_JUMP_R_STRAIGHT

    orig_dl = dleft_ticks
    orig_dr = dright_ticks

    dleft_ticks = clamp(dleft_ticks, -limit_l, limit_l)
    dright_ticks = clamp(dright_ticks, -limit_r, limit_r)

    if dleft_ticks != orig_dl or dright_ticks != orig_dr:
        print("[ODOM] Clamped tick jump: dL={}->{}, dR={}->{}".format(
            int(orig_dl), int(dleft_ticks), int(orig_dr), int(dright_ticks)
        ))

    left_dist_per_tick = (math.pi * WHEEL_DIAMETER_M) / LEFT_TICKS_PER_REV
    right_dist_per_tick = (math.pi * WHEEL_DIAMETER_M) / RIGHT_TICKS_PER_REV

    dl = dleft_ticks * left_dist_per_tick
    dr = dright_ticks * right_dist_per_tick

    dc = (dl + dr) / 2.0
    dth = (dr - dl) / WHEEL_BASE_M

    if abs(dc) > MAX_LINEAR_STEP_M or abs(dth) > MAX_ANGULAR_STEP_RAD:
        print("[ODOM] Rejected motion step: dc={:.4f} m, dth={:.4f} rad".format(dc, dth))
        return

    pose_x += dc * math.cos(pose_th + dth / 2.0)
    pose_y += dc * math.sin(pose_th + dth / 2.0)
    pose_th = normalize_angle(pose_th + dth)

    vx = dc / dt
    vth = dth / dt

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


def run_feeding_cycle(trigger_source="manual"):
    global feeding_in_progress, last_feed_date

    with feed_lock:
        if feeding_in_progress:
            print("[FEED] Feeding already in progress")
            broadcast_to_clients("LOG:Feeding already in progress")
            return
        feeding_in_progress = True

    try:
        today = datetime.now().strftime("%Y-%m-%d")
        broadcast_to_clients("FEEDING:START")
        broadcast_to_clients("LOG:Feeding cycle started ({})".format(trigger_source))

        for i in range(10):
            if rospy.is_shutdown():
                break

            feed_ack_event.clear()
            send_uart(ser_global, "<K>")
            broadcast_to_clients("FEEDING:STEP:{}/10".format(i + 1))
            broadcast_to_clients("LOG:Sent K {}/10".format(i + 1))

            ack_ok = feed_ack_event.wait(timeout=7.0)
            if ack_ok:
                broadcast_to_clients("LOG:Ack received for step {}/10".format(i + 1))
            else:
                broadcast_to_clients("LOG:Ack timeout for step {}/10".format(i + 1))

        last_feed_date = today
        save_feed_config()
        broadcast_to_clients("FEEDING:DONE")
        broadcast_to_clients("LOG:Feeding cycle completed")
        send_status_to_client()

    except Exception as e:
        print("[FEED ERROR]", e)
        broadcast_to_clients("ERROR:Feeding failed: {}".format(e))
    finally:
        with feed_lock:
            feeding_in_progress = False


def feeder_scheduler_loop():
    global last_feed_date

    print("[FEED] Scheduler started")

    while not rospy.is_shutdown():
        try:
            now = datetime.now()
            now_hhmm = now.strftime("%H:%M")
            today = now.strftime("%Y-%m-%d")

            if (now_hhmm == feed_time_hhmm) and (last_feed_date != today) and (not feeding_in_progress):
                print("[FEED] Scheduled feed triggered at", now_hhmm)
                t = threading.Thread(target=run_feeding_cycle, args=("scheduled",))
                t.daemon = True
                t.start()
                time.sleep(61)
                continue

        except Exception as e:
            print("[FEED SCHED ERROR]", e)

        time.sleep(1.0)


def time_broadcast_loop():
    while not rospy.is_shutdown():
        try:
            broadcast_to_clients("TIME:" + datetime.now().strftime("%H:%M:%S"))
        except Exception:
            pass
        time.sleep(1.0)


def serial_reader(ser, odom_pub, tf_broadcaster):
    print("[UART] Reader started")

    while not rospy.is_shutdown():
        try:
            raw = ser.readline()

            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line == "O" or line == "<O>" or line == "ACK:O":
                print("[MEGA] feeder ack:", line)
                feed_ack_event.set()
                broadcast_to_clients("ACK:O")
                continue

            parsed = parse_odo_packet(line)
            if parsed is None:
                print("[MEGA] ignored:", line)
                continue

            left, right = parsed
            print("[MEGA] <ODO,{},{:d}>".format(left, right))

            publish_odom(odom_pub, tf_broadcaster, left, right)

        except Exception as e:
            print("[UART ERROR]", e)
            time.sleep(0.05)


def extract_cmd_payload(msg):
    if not msg.startswith("<CMD:") or not msg.endswith(">"):
        return None

    payload = msg[5:-1].strip()
    if not payload:
        return None

    return payload.upper()


def run_timed_motion(cmd, duration, resend_interval=0.10):
    global auto_cancel, dog_found

    end_time = time.time() + duration

    while time.time() < end_time and not rospy.is_shutdown():
        if auto_cancel or dog_found:
            stop_robot()
            return False

        try:
            send_uart(ser_global, "<{}>".format(cmd))
        except Exception as e:
            print("[AUTO ERROR] Failed to send {}: {}".format(cmd, e))
            stop_robot()
            return False

        time.sleep(resend_interval)

    stop_robot()
    time.sleep(0.15)
    return True


def autonomous_sequence():
    global auto_running, auto_cancel, dog_found

    with auto_lock:
        if auto_running:
            print("[AUTO] Already running")
            return
        auto_running = True

    try:
        auto_cancel = False
        dog_found = False

        print("[AUTO] Starting sequence")

        print("[AUTO] Step 1: forward 5s")
        if not run_timed_motion("F", 5.0):
            print("[AUTO] Stopped during step 1")
            return

        print("[AUTO] Step 2: clockwise turn ~90 deg")
        if not run_timed_motion("D", AUTO_TURN_90_TIME):
            print("[AUTO] Stopped during step 2")
            return

        print("[AUTO] Step 3: forward 3s")
        if not run_timed_motion("F", 3.0):
            print("[AUTO] Stopped during step 3")
            return

        stop_robot()
        print("[AUTO] Sequence complete")

    finally:
        with auto_lock:
            auto_running = False
        auto_cancel = False


def handle_client(conn, addr, ser):
    global auto_cancel, feed_time_hhmm

    print("[NET] Client connected:", addr)

    with clients_lock:
        clients.append(conn)

    try:
        send_status_to_client(conn)

        while not rospy.is_shutdown():
            data = conn.recv(1024)
            if not data:
                break

            chunks = data.decode("ascii", "ignore").splitlines()
            for raw_msg in chunks:
                msg = raw_msg.strip()
                if not msg:
                    continue

                print("[NET RX]", msg)

                if msg.startswith("SET_FEED_TIME:"):
                    hhmm = msg.split(":", 1)[1].strip()
                    try:
                        time.strptime(hhmm, "%H:%M")
                        feed_time_hhmm = hhmm
                        save_feed_config()
                        conn.sendall(("FEED_TIME:{}\n".format(feed_time_hhmm)).encode("ascii", "ignore"))
                        conn.sendall(b"LOG:Feed time updated\n")
                        send_status_to_client()
                    except Exception:
                        conn.sendall(b"ERROR:Invalid feed time\n")
                    continue

                if msg == "GET_STATUS":
                    send_status_to_client(conn)
                    continue

                if msg == "FEED_NOW":
                    t = threading.Thread(target=run_feeding_cycle, args=("manual",))
                    t.daemon = True
                    t.start()
                    conn.sendall(b"LOG:Manual feeding requested\n")
                    continue

                cmd = extract_cmd_payload(msg)
                if cmd is None:
                    conn.sendall("<ERR>\n".encode("ascii", "ignore"))
                    continue

                if cmd == "L":
                    cmd = "D"

                if cmd == "AUTO":
                    print("[NET] Received mode command: AUTO")
                    auto_cancel = False

                    t = threading.Thread(target=autonomous_sequence)
                    t.daemon = True
                    t.start()

                    conn.sendall("<ACK:AUTO>\n".encode("ascii", "ignore"))
                    continue

                if cmd == "K":
                    t = threading.Thread(target=run_feeding_cycle, args=("legacy_cmd_k",))
                    t.daemon = True
                    t.start()
                    conn.sendall("<ACK:K>\n".encode("ascii", "ignore"))
                    continue

                if cmd in VALID_UART_COMMANDS:
                    auto_cancel = True
                    send_uart(ser, "<{}>".format(cmd))
                    conn.sendall("<ACK:{}>\n".format(cmd).encode("ascii", "ignore"))
                elif cmd in VALID_WORD_COMMANDS:
                    conn.sendall("<ACK:{}>\n".format(cmd).encode("ascii", "ignore"))
                else:
                    conn.sendall("<ERR>\n".encode("ascii", "ignore"))

    except Exception as e:
        print("[NET ERROR]", e)

    finally:
        auto_cancel = True
        stop_robot()

        with clients_lock:
            try:
                clients.remove(conn)
            except ValueError:
                pass

        conn.close()
        print("[NET] Client disconnected:", addr)


def main():
    rospy.init_node("jcserver_mapping_ros")

    load_feed_config()

    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=20)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=10)
    rospy.Subscriber("/dog_detected", Bool, dog_detected_callback, queue_size=5)

    tf_broadcaster = tf.TransformBroadcaster()

    print("[SYS] Starting jcserver_mapping_ros...")

    global ser_global
    ser = serial.Serial(UART_DEV, UART_BAUD, timeout=0.05)
    ser_global = ser
    time.sleep(0.5)

    print("[UART] Opened", UART_DEV)
    print("[FEED] Current feed time:", feed_time_hhmm)

    t1 = threading.Thread(target=serial_reader, args=(ser, odom_pub, tf_broadcaster))
    t1.daemon = True
    t1.start()

    t2 = threading.Thread(target=feeder_scheduler_loop)
    t2.daemon = True
    t2.start()

    t3 = threading.Thread(target=time_broadcast_loop)
    t3.daemon = True
    t3.start()

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

        t4 = threading.Thread(target=handle_client, args=(conn, addr, ser))
        t4.daemon = True
        t4.start()


if __name__ == "__main__":
    main()
