package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Const;


public class Intake {

    //wrist servo - twist
    //jaw servo
    // wrist servo - lateral movement
    // vex motor turns intake fingers

    public Servo lateralIntakeServo;
    public Servo twistIntake;
    public Servo jawServo;
    public CRServo intake;




    public Intake(HardwareMap hardwareMap) {

        intake = hardwareMap.get(CRServo.class, "intake");
        lateralIntakeServo = hardwareMap.get(Servo.class, "lateralIntakeServo");
        twistIntake = hardwareMap.get(Servo.class, "twistIntake");
        jawServo = hardwareMap.get(Servo.class, "jawServo");

        //TODO: add directions and starting positions




        lateralIntakeServo.setPosition(Constants.LATERAL_INTAKE_SERVO_INIT);
        jawServo.setPosition(Constants.JAW_SERVO_INIT);
        twistIntake.setPosition(Constants.INTAKE_TWISTER_INIT);
    }

    public void setIntakeSpeed(double intakeSpeed) { intake.setPower(intakeSpeed); }

    public void setjawPosition(double jawPosition) { jawServo.setPosition(jawPosition); }

    public void setTwistPosition(double twistPosition) { twistIntake.setPosition(twistPosition); }

    public void setLateralIntakePosition(double lateralIntakePosition) { lateralIntakeServo.setPosition(lateralIntakePosition); }









}
