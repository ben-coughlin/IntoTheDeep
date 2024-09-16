package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBars {
    public Servo left4Bar;
    public Servo right4Bar;

    private double lastSetPosition;

    private double fourBarActionPosition; // variable for RR1.0 actions
    private double fourBarPosition; // variable for regular use like on init.. humm may not need since its defined in the method?

    public FourBars(HardwareMap hardwareMap) {
        left4Bar = hardwareMap.get(Servo.class, "fourBarLeft");
        right4Bar = hardwareMap.get(Servo.class, "fourBarRight");
        right4Bar.setDirection(Servo.Direction.REVERSE);
    }


    /////////////////////////   BOTH 4 BARS  /////////////////////////////////////
    public class FourBarActionSet implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                left4Bar.setPosition(fourBarActionPosition);
                right4Bar.setPosition(fourBarActionPosition);
                return  false;
            }

    }

    public Action fourBarActionSet(double fourBarActionSetPosition) {  // this method is for use in RR trajectories
        fourBarActionPosition = fourBarActionSetPosition;
        return new FourBarActionSet();
    }

    public class FourBarActionHome implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            left4Bar.setPosition(.85);
            right4Bar.setPosition(.85);
            return  false;
        }

    }
    public Action fourBarActionSHome() {  // this method is for use in RR trajectories

        return new FourBarActionHome();
    }
    public class FourBarActionExtended implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            left4Bar.setPosition(0);
            right4Bar.setPosition(0);
            return  false;
        }

    }

    public class FourBarActionPrep implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            left4Bar.setPosition(.4);
            right4Bar.setPosition(.4);
            return  false;
        }

    }

    public Action fourBarActionExtended() {  // this method is for use in RR trajectories

        return new FourBarActionExtended();
    }

    public Action fourBarActionPrep() {  // this method is for use in RR trajectories

        return new FourBarActionPrep();
    }

    public void setFourBarPosition(double fourBarPosition){  // this method is for basic use like init...
        lastSetPosition = fourBarPosition;
        left4Bar.setPosition(fourBarPosition);
        right4Bar.setPosition(fourBarPosition);
    }

    public double getLastSetPosition() {
        return lastSetPosition;
    }
}
