package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarRotator {
    public Servo fourBarRotator;

    private double fourBarRotatorActionPosition; // variable for RR1.0 actions
    private double fourBarRotatorPosition; // variable for regular use like on init.. humm may not need since its defined in the method?


    public FourBarRotator(HardwareMap hardwareMap) {
        fourBarRotator = hardwareMap.get(Servo.class, "fourBarRotator");
        fourBarRotator.setDirection(Servo.Direction.REVERSE);

    }

    /////////////////////////  Four Bar Rotator   /////////////////////////////////////
    public class FourBarRotatorActionSet implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                fourBarRotator.setPosition(fourBarRotatorActionPosition);
                return false;

        }
    }
    public class FourBarRotatorActionScore implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            fourBarRotator.setPosition(.45);
            return false;

        }
    }
    public class FourBarRotatorActionHome implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            fourBarRotator.setPosition(.82);
            return false;

        }
    }

    public Action fourBarRotatorActionSet(double fourBarRotatorActionSetPosition) {  // this method is for use in RR trajectories
        fourBarRotatorActionPosition = fourBarRotatorActionSetPosition;
        return new FourBarRotatorActionSet();
    }
    public Action fourBarRotatorActionScore() {  // this method is for use in RR trajectories
        return new FourBarRotatorActionScore();
    }
    public Action fourBarRotatorActionHome() {  // this method is for use in RR trajectories
        return new FourBarRotatorActionHome();
    }

    public void setFourBarRotatorPosition(double fourBarRotatorPosition){  // this method is for basic use like init...
        fourBarRotator.setPosition(fourBarRotatorPosition);
    }


}
