package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotorEx liftLeft = null;
    public DcMotorEx liftRight = null;

    public int liftPosition = 0;

    private double minPowerLift = 0;
    private double maxPowerLift = 1;

    private double pivotMaxTicks = 0;

    PController liftPController = new PController(0.002);

    public Lift(HardwareMap hardwareMap) {

        liftRight = hardwareMap.get(DcMotorEx.class, "rightPivot");
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setPower(0);
        //pixelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft = hardwareMap.get(DcMotorEx.class, "leftPivot");
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setPower(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void InitLiftPController(){

        liftPController.setInputRange(0, pivotMaxTicks);
        liftPController.setSetPoint(0);
        liftPController.setOutputRange(minPowerLift, maxPowerLift);
        liftPController.setThresholdValue(5);
        // this is for lift left, change Kp to calibrate
    }

    public void updateLiftPosition() {

        //todo: should one of these getpos be negative??
        liftPosition = (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition());

        if (liftPosition < liftPController.setPoint) {

            liftLeft.setPower(minPowerLift +
                    liftPController.getComputedOutput(liftPosition));
            liftRight.setPower(minPowerLift +
                    liftPController.getComputedOutput(liftPosition));
        } else {
            liftLeft.setPower(minPowerLift -
                    liftPController.getComputedOutput(liftPosition));
            liftRight.setPower(minPowerLift -
                    liftPController.getComputedOutput(liftPosition));
        }
    }
}
