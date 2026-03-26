def cmd_vel_callback(msg):
    global last_cmd_sent, last_cmd_time

    lin = msg.linear.x
    ang = msg.angular.z

    if abs(lin) < CMD_DEADBAND_LINEAR and abs(ang) < CMD_DEADBAND_ANGULAR:
        cmd = "S"
    elif abs(ang) >= CMD_DEADBAND_ANGULAR:
        if ang > 0:
            cmd = "A"   # CCW / left
        else:
            cmd = "D"   # CW / right
    elif lin > CMD_DEADBAND_LINEAR:
        cmd = "F"
    elif lin < -CMD_DEADBAND_LINEAR:
        cmd = "R"
    else:
        cmd = "S"

    now = time.time()

    with cmd_vel_lock:
        # avoid spamming identical commands too fast
        if cmd != last_cmd_sent or (now - last_cmd_time) >= CMD_REPEAT_INTERVAL:
            try:
                send_uart(ser_global, "<{}>".format(cmd))
                last_cmd_sent = cmd
                last_cmd_time = now
                print("[CMD_VEL] lin={:.3f}, ang={:.3f} -> {}".format(lin, ang, cmd))
            except Exception as e:
                print("[CMD_VEL ERROR]", e)



ser_global = None


    global ser_global
    ser = serial.Serial(UART_DEV, UART_BAUD, timeout=0.05)
    ser_global = ser


    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=10)

