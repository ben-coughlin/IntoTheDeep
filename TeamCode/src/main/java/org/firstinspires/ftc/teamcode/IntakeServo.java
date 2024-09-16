package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeServo {
    public ServoImplEx intakeServo;

    public IntakeServo(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(ServoImplEx.class, "intakeExtender");
        intakeServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setPosition(0.01);
    }
}
