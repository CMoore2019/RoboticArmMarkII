def MoveTo(positionvector,theta5):
    # function that takes a row vector of joint angles in degrees and causes the robot to move to the corresponding position
    # Returns 0 if everything is fine, 1 if a collision is detected.
    # baseangle range: -135 to 135
    # shoulderangle range: -135 to 135
    # elbowangle range: -135 to 135
    # wristangle range: -90 to 90
    # clawangle range: 0 to 155
    # Input: [baseangle, shoulderangle, elbowangle, wristangle, clawangle]
    from workspacetest import workspacetest
    from FK import FK
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)
    
    if workspacetest(FK([positionvector[0],positionvector[1],positionvector[2],positionvector[3]])) == False:
        print("Collision Detected. Skipping this orientation.")
        return 1
    
    baseangle = positionvector[0]+135
    shoulderangle = positionvector[1]+135
    elbowangle = positionvector[2]+135
    wristangle = positionvector[3]+90
    clawangle = theta5+25

    # Designate channels
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    WRIST = 3
    CLAW = 4

    # For the Mark II BASE the min angle is 0 degrees, and the max is 270 degrees
    # For the Mark II SHOULDER the min angle is 30 degrees, and the max is 240 degrees
    # For the Mark II ELBOW the min angle is 20 degrees, and the max is 250 degrees
    # For the Mark II WRIST the min angle is 0 degrees, and the max is 180 degrees
    # For the Mark II CLAW the min angle is 25 degrees, and the max is 180 degrees

    # Pulse Limitations:
    # For the Mark II BASE the range is (500, 2700) and actuation range is 270
    # For the Mark II SHOULDER the range is (500, 2700) and actuation range is 270
    # For the Mark II ELBOW the range is (500, 2700) and the actuation range is 270
    # For the Mark II WRIST the range is (400, 2400) and the actuation range is 180
    # For the Mark II CLAW the range is (400, 2700) and the actuation range is 180
    
    # BASE
    kit.servo[BASE].set_pulse_width_range(500, 2700)
    kit.servo[BASE].actuation_range = 270
    # SHOULDER
    kit.servo[SHOULDER].set_pulse_width_range(500, 2700)
    kit.servo[SHOULDER].actuation_range = 270
    # ELBOW
    kit.servo[ELBOW].set_pulse_width_range(500, 2700)
    kit.servo[ELBOW].actuation_range = 270
    # WRIST
    kit.servo[WRIST].set_pulse_width_range(400, 2400)
    kit.servo[WRIST].actuation_range = 180
    # CLAW
    kit.servo[CLAW].set_pulse_width_range(400, 2700)
    kit.servo[CLAW].actuation_range = 180

    kit.servo[BASE].angle = baseangle
    kit.servo[SHOULDER].angle = shoulderangle
    kit.servo[ELBOW].angle = elbowangle
    kit.servo[WRIST].angle = wristangle
    kit.servo[CLAW].angle = clawangle
    return 0