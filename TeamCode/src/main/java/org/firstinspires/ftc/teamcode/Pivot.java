package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pivot {

    public DcMotorEx leftPivot = null;
    public DcMotorEx rightPivot = null;

    public int pivotPosition = 0;

    private double minPowerPivot = 0;
    private double maxPowerPivot = 1;

    private double pivotMaxTicks = 0;

    PController pivotPController = new PController(0.002);

    public Pivot(HardwareMap hardwareMap) {
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");
        rightPivot.setDirection(DcMotor.Direction.REVERSE);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPivot.setPower(0);
        //pixelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPivot.setPower(0);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void InitPivotPController(){

        pivotPController.setInputRange(0, pivotMaxTicks);
        pivotPController.setSetPoint(0);
        pivotPController.setOutputRange(minPowerPivot, maxPowerPivot);
        pivotPController.setThresholdValue(5);
        // this is for lift left, change Kp to calibrate
    }

    public void updatePivotPosition() {

        //todo: should one of these getpos be negative??
        pivotPosition= (leftPivot.getCurrentPosition() + rightPivot.getCurrentPosition());

        if (pivotPosition < pivotPController.setPoint) {

            leftPivot.setPower(minPowerPivot +
                    pivotPController.getComputedOutput(pivotPosition));
            rightPivot.setPower(minPowerPivot +
                    pivotPController.getComputedOutput(pivotPosition));
        } else {
            leftPivot.setPower(minPowerPivot -
                    pivotPController.getComputedOutput(pivotPosition));
            rightPivot.setPower(minPowerPivot -
                    pivotPController.getComputedOutput(pivotPosition));
        }
    }
}
