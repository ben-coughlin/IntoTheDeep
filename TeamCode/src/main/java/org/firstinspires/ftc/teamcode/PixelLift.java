package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixelLift {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    public DcMotorEx pixelLift;
    public int pixelLiftPosition = 0;
    private double pixelLiftMaxTicks = 2370;
    // for lift pController input
    private double minPowerPixelLift = .1;
    private double maxPowerPixelLift = 0.98;
    private double pixelLiftActionPosition; // variable for RR1.0 actions
    //private double pixelLiftPosition; // variable for regular use like on init.. humm may not need since its defined in the method?
    private boolean exitPixelLiftPControllerLoop = false;
    public PController pControllerPixelLift = new PController(0.002);

    public PixelLift(HardwareMap hardwareMap) {
        pixelLift = hardwareMap.get(DcMotorEx.class, "lift");
        pixelLift.setDirection(DcMotor.Direction.REVERSE);
        pixelLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixelLift.setPower(0);
        //pixelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void InitPixelLiftPIDController(){

        pControllerPixelLift.setInputRange(0, pixelLiftMaxTicks);
        pControllerPixelLift.setSetPoint(0);
        pControllerPixelLift.setOutputRange(minPowerPixelLift, maxPowerPixelLift);
        pControllerPixelLift.setThresholdValue(5);
        // this is for lift left, change Kp to calibrate
    }

    public void updateLiftPosition() {
        pixelLiftPosition=pixelLift.getCurrentPosition();

        if (pixelLiftPosition < pControllerPixelLift.setPoint) {

            pixelLift.setPower(minPowerPixelLift +
                    pControllerPixelLift.getComputedOutput(pixelLiftPosition));
        } else {
            pixelLift.setPower(minPowerPixelLift -
                    pControllerPixelLift.getComputedOutput(pixelLiftPosition));
        }
    }


    /////////////////////////   PixelLift   /////////////////////////////////////
    public class PixelLiftElevationMaintainer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


                pixelLiftPosition=pixelLift.getCurrentPosition();

                if (pixelLiftPosition < pControllerPixelLift.setPoint) {

                    pixelLift.setPower(minPowerPixelLift +
                            pControllerPixelLift.getComputedOutput(pixelLiftPosition));
                } else {
                    pixelLift.setPower(minPowerPixelLift -
                            pControllerPixelLift.getComputedOutput(pixelLiftPosition));
                }
                return true;

        }
    }
    public Action pixelLiftElevationMaintainer() {  // this method is for use in RR trajectories
        return new PixelLiftElevationMaintainer();
    }

    public Action setPixelLiftPosition(int pixelLiftActionSetPosition) {  // this method is for use in RR trajectories
        pixelLiftActionPosition = pixelLiftActionSetPosition;
        return new PixelLiftActionSet();
    }

    public class PixelLiftActionSet implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerPixelLift.setSetPoint(pixelLiftActionPosition);
            return false;

        }
    }
    public class PixelLiftActionHome implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerPixelLift.setSetPoint(0);
            return false;

        }
    }
    public Action setPixelLiftActionHome() {  // this method is for use in RR trajectories

        return new PixelLiftActionHome();
    }
        public class PixelLiftActionLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                pControllerPixelLift.setSetPoint(600);
                return false;

            }
    }
    public Action setPixelLiftActionLow() {  // this method is for use in RR trajectories

        return new PixelLiftActionLow();
    }
    public class PixelLiftActionHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerPixelLift.setSetPoint(700);
            return false;

        }
    }
    public Action setPixelLiftActionHigh() {  // this method is for use in RR trajectories

        return new PixelLiftActionHigh();
    }





}
