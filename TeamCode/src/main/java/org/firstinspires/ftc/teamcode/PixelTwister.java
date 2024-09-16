package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelTwister {
    public Servo pixelTwister;

    public double lastPixelTwisterPosition = 0;

    private double pixelTwisterActionPosition; // variable for RR1.0 actions
    private double pixelTwisterPosition;  // variable for regular use like on init.. humm may not need since its defined in the method?

    public PixelTwister(HardwareMap hardwareMap) {
        pixelTwister = hardwareMap.get(Servo.class, "pixelRotator");

    }

    /////////////////////////  PixelTwister   /////////////////////////////////////
    public class PixelTwisterActionSet implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pixelTwister.setPosition(pixelTwisterActionPosition);
            return false;

        }
    }
    public class PixelTwisterActionOne implements Action {


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pixelTwister.setPosition(1);
            return false;

        }
    }
    public class PixelTwisterActionHome implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pixelTwister.setPosition(.49);
            return false;

        }
    }
    public class PixelTwisterActionZero implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pixelTwister.setPosition(0);
            return false;

        }
    }

    public Action pixelTwisterActionSet(double pixelTwisterActionSetPosition) {  // this method is for use in RR trajectories
        pixelTwisterActionPosition = pixelTwisterActionSetPosition;
        return new PixelTwisterActionSet();
    }
    public Action pixelTwisterActionOne() {  // this method is for use in RR trajectories
         return new PixelTwisterActionOne();
    }
    public Action pixelTwisterActionZero() {  // this method is for use in RR trajectories
        return new PixelTwisterActionZero();
    }
    public Action pixelTwisterActionHome() {  // this method is for use in RR trajectories
        return new PixelTwisterActionHome();
    }

    public void setPixelTwisterPosition(double pixelTwisterPosition) {  // this method is for basic use like init...
        lastPixelTwisterPosition = pixelTwisterPosition;
        pixelTwister.setPosition(pixelTwisterPosition);
    }

    public double getLastIntakePosition() {
        return lastPixelTwisterPosition;
    }

}



