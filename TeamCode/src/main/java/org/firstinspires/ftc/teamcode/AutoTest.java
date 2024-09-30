package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoTest extends AutoMaster {
    private final double SCALE_FACTOR = 1.0;

    private int cycles = 0;

    double endXatBackboard = 31;
    double endYatBackboard = -10; //-

    boolean flag = true;
    long startTime;

    public enum progStates {
        trustSpikeLocationState1,
        trustSpikeLocationState2,
        placePixel,
        backupFromPixel,
        driveToBackBoard,
        dropYellowPixel,
        driveToPixelStack,
        pikcupPixelStack,
        driveToBackboardDelivery,
        park,
    }

    @Override
    public void init() {
        super.init();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void init_loop() {
        //List<Recognition> currentRecognitions = tfod.getRecognitions();
        //telemetry.addData("# Objects Detected", currentRecognitions.size());
//        if (currentRecognitions.size() == 0) {
//            spikeLocation = 3;
//        }
        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop() + recognition.getBottom()) / 2;
//
//            if (x <= 90) {
//                spikeLocation = 3;
//            } else if (x < 400 && x > 90) {
//                spikeLocation = 2;
//            } else
//                spikeLocation = 1;
//
//            //visionPortal.resumeStreaming();  // may not need this since we will be turning it off at start
//
//            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//            telemetry.addData(">", "Touch Play to start OpMode");
//            telemetry.addData("", " ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
        telemetry.addData("spike location", spikeLocation);
        telemetry.update(); // this also Pushes vision portal telemetry to the Driver Station... i think
    }

    @Override
    public void start() {
        super.start();

//        fourBars.setFourBarPosition(0.4);
//        pixelLift.pControllerPixelLift.setSetPoint(600);
//
//        if (spikeLocation == 3) {
//            programStage = progStates.trustSpikeLocationState1.ordinal();
//        } else {
//            programStage = progStates.placePixel.ordinal();
//        }
    }

    @Override
    public void mainLoop() {
        telemetry.addData("Current State", programStage);

        if (programStage == progStates.trustSpikeLocationState1.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(15, 0,
                    0.8 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

            drive.applyMovementDirectionBased();

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.trustSpikeLocationState2.ordinal());
            }
        }

        if (programStage == progStates.trustSpikeLocationState2.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-19, 0,
                    0.4 * SCALE_FACTOR, 0.5 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

            drive.applyMovementDirectionBased();

            // drive forward a bit and set four bar location

//            if (worldXPosition < -5) {
//                fourBars.setFourBarPosition(0);
//                droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(.75);
//
//            }

// this sets the lift height at the start of this state
//            pixelLift.pControllerPixelLift.setSetPoint(350);
//            pixelLift.updateLiftPosition();
//
//            // set pixel twister around for dropping purple pixel when the lift gets grater than 600
//            if (pixelLift.pixelLiftPosition > 275) {
//                pixelTwister.setPixelTwisterPosition(0);
//            }

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                grippers.frontGripper.setPosition(.16);
                nextStage(progStates.backupFromPixel.ordinal());
            }
        }

        if (programStage == progStates.placePixel.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            double targetX = spikeLocation == 3 ? -15 : -10;
            double targetY = spikeLocation == 3 ? -2 : 0;

            points.add(new CurvePoint(targetX, targetY,
                    0.8 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

// define end points for various spike locations
            double endX = -21;
            double endY = 6.5; //was 6.5

            if (spikeLocation == 1) {
                endX = -13.25;
                endY = 1.875;//was 2.75
            } else if (spikeLocation == 2) {
                endX = -20;
                endY = 0.125;//was 1
            } else if (spikeLocation == 3) {
                endX = -14;
                endY = -6.25;//was -4.75
            }
// drive to spike locations based on end x and end y definitions
            points.add(new CurvePoint(endX, endY,
                    0.1 * SCALE_FACTOR, 0.2 * SCALE_FACTOR, 10, 10,
                    Math.toRadians(60), 0.6));

            drive.applyMovementDirectionBased();

            // drive forward a bit and set four bar location

            if (worldXPosition < -5) {
                fourBars.setFourBarPosition(0);
                droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(.75);

            }

// this sets the lift height at the start of this state
            pixelLift.pControllerPixelLift.setSetPoint(350);
            pixelLift.updateLiftPosition();

            // set pixel twister around for dropping purple pixel when the lift gets grater than 600
            if (pixelLift.pixelLiftPosition > 275) {
                pixelTwister.setPixelTwisterPosition(0);
            }

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                grippers.frontGripper.setPosition(.16);
                nextStage();
            }
        }

// this backs awy from pikes so we don't drive over purple pixel
        if (programStage == progStates.backupFromPixel.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            /*ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX,stateStartingY,
                    0, 0, 0, 0, 0, 0));

            // this point backs away from pixel
            points.add(new CurvePoint(8,0,
                    0.8*SCALE_FACTOR, 0.9*SCALE_FACTOR,20,20,
                    Math.toRadians(60),0.6));*/

            //wait some time before applying the movement
            if (SystemClock.uptimeMillis() - stateStartTime > 600) {
                movement_y = 0.6;
                drive.applyMovementDirectionBased();
            } else {
                drive.stopAllMovementDirectionBased();
            }

            if (SystemClock.uptimeMillis() - stateStartTime > 1075) {
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }

// drive to backboard
        if (programStage == progStates.driveToBackBoard.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                pixelTwister.setPixelTwisterPosition(.49);
                fourBarRotator.fourBarRotator.setPosition(.45);
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            //define where to drive to place yellow pixel

            if (spikeLocation == 1) {
                endXatBackboard = -16.5;
            } else if (spikeLocation == 2) {
                endXatBackboard = -24.5;
                //endYatBackboard = 1;
            } else if (spikeLocation == 3) {
                endXatBackboard = -29;
                //endYatBackboard = -6.5;
            }

            // get close to backboard but not all the way
            points.add(new CurvePoint(endXatBackboard, 10,
                    0.8 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6)); //y was 10

            //get to closer placement position
            points.add(new CurvePoint(endXatBackboard, 25,
                    0.8 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6)); //y was 25

            if (spikeLocation != 3) {
                points.add(new CurvePoint(endXatBackboard, 30,
                        0.8 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 15,
                        Math.toRadians(60), 0.6)); //y was 30
            }

            //get to closer placement position
            points.add(new CurvePoint(endXatBackboard, 36,
                    0.65 * SCALE_FACTOR, 0.65 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6)); //y was 36.75

            drive.applyMovementDirectionBased();

            //sets lift position at the start of this state
            pixelLift.pControllerPixelLift.setSetPoint(600);
            pixelLift.updateLiftPosition();

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                grippers.backGripper.setPosition(.21); // dropping of the pixel
                nextStage();
            }
        }

        if (programStage == progStates.dropYellowPixel.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                drive.stopAllMovementDirectionBased();
                grippers.frontGripper.setPosition(.16);
                grippers.backGripper.setPosition(.21);
            }

            /*ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX,stateStartingY,
                    0, 0, 0, 0, 0, 0));

            //back away from backdrop
            points.add(new CurvePoint(stateStartingX,stateStartingY-5,
                    0.9*SCALE_FACTOR, 0.9*SCALE_FACTOR,10,15,
                    Math.toRadians(60),0.6));*/

            pixelLift.updateLiftPosition();

            if (SystemClock.uptimeMillis() - stateStartTime > 500) {
                movement_y = 0.3;//was 0.8
                drive.applyMovementDirectionBased();
                if (SystemClock.uptimeMillis() - stateStartTime > 750) {
                    drive.stopAllMovementDirectionBased();
                    nextStage();
                }
            } else {
                movement_y = -0.18;//was -.18
                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveToPixelStack.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                pixelTwister.setPixelTwisterPosition(.48);
                fourBarRotator.setFourBarRotatorPosition(.83);
                fourBars.setFourBarPosition(.4);
                if (cycles >= 1) {
                    intakeServo.intakeServo.setPosition(0.25);
                    intake.intake.setPower(1);
                } else {
                    intakeServo.intakeServo.setPosition(0.01);
                }
                //intakeServo.intakeServo.setPwmDisable();

                startTime = SystemClock.uptimeMillis();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            /*points.add(new CurvePoint(stateStartingX, stateStartingY - 10,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6));*/

            double targetX = -52; // was 54
            double targetY = spikeLocation == 1 ? 20 : 5;

            points.add(new CurvePoint(-57, targetY,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(targetX, targetY - 10,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(targetX, -50,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(targetX, -55,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6)); //was -62

            double endY = 0;//cycles == 0 ? -67 : -68.5;

            if (cycles == 0) {
                endY = -70;
            } else if (cycles == 1) {
                endY = -68.5;
            } else {
                endY = -71.5;
            }

            points.add(new CurvePoint(targetX, endY,
                    0.15 * SCALE_FACTOR, 0.15 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6));

            if (SystemClock.uptimeMillis() - stateStartTime > 500) {   //wait till y position is less than 10 to lower lift to give time for grabber servos to get into safe position
                pixelLift.pControllerPixelLift.setSetPoint(0);
                pixelLift.updateLiftPosition();
            }

            if (worldYPosition < 0) {
                //intake.intake.setPower(1);
                intake.intake.setPower(-.91);
            }

            drive.applyMovementDirectionBased();

            /*long currTime = SystemClock.uptimeMillis();
            long elapsedTime = currTime - startTime;

            if (elapsedTime >= 200) {
                startTime = currTime;
                flag = !flag;

                if (flag) {
                    intakeServo.intakeServo.setPosition(0.25);
                } else {
                    intakeServo.intakeServo.setPosition(0.1);
                }
            }*/

            if (Movement.followCurve(points, Math.toRadians(90))) {
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }

        if (programStage == progStates.pikcupPixelStack.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                //intakeServo.intakeServo.setPwmEnable();
                if (cycles == 0) {
                    intakeServo.intakeServo.setPosition(0.335);
                    intake.intake.setPower(0.15);
                } else {
                    intakeServo.intakeServo.setPosition(0.335);
                    intake.intake.setPower(1);
                }

                startTime = SystemClock.uptimeMillis();
            }

            if (SystemClock.uptimeMillis() - stateStartTime < 1750 && SystemClock.uptimeMillis() - stateStartTime > 350) {
                long currTime = SystemClock.uptimeMillis();
                long elapsedTime = currTime - startTime;

                if (SystemClock.uptimeMillis() - stateStartTime > 1000) {
                    movement_y = 0;
                    intake.intake.setPower(1);
                } else {
                    movement_y = -0.18;
                }

                drive.applyMovementDirectionBased();

                if (SystemClock.uptimeMillis() - stateStartTime > 1100 || (cycles > 0 && SystemClock.uptimeMillis() - stateStartTime > 360)) {
                    if (elapsedTime >= 250) {
                        startTime = currTime;
                        flag = !flag;

                        if (flag) {
                            intakeServo.intakeServo.setPosition(0.335);
                        } else {
                            intakeServo.intakeServo.setPosition(0.18);
                        }
                    }
                }
            }

            long timeout = 0; //cycles == 0 ? 1750 : 1000; ///  2200 : 1200

            if (cycles == 0) {
                timeout = 2250;
            } else if (cycles == 1) {
                timeout = 1000;
            }

            if (SystemClock.uptimeMillis() - stateStartTime < 3000 && SystemClock.uptimeMillis() - stateStartTime > 1750) {
                intake.intake.setPower(1);
                movement_y = 0.18;
                drive.applyMovementDirectionBased();
            }

            if (SystemClock.uptimeMillis() - stateStartTime > timeout) {
                intakeServo.intakeServo.setPosition(0.25);
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }

        // this state moves to the backboard
        if (programStage == progStates.driveToBackboardDelivery.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(-55, -4,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6)); //y was -4

            points.add(new CurvePoint(-52, 9,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 20, 24,
                    Math.toRadians(60), 0.6));//y was 9

            points.add(new CurvePoint(-28, 28,
                    0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6));//y was 30

            points.add(new CurvePoint(-28, 37.25 - 8,
                    0.05 * SCALE_FACTOR, 0.05 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6));//y was 37.25

            points.add(new CurvePoint(-28, 36,
                    0.05 * SCALE_FACTOR, 0.05 * SCALE_FACTOR, 10, 15,
                    Math.toRadians(60), 0.6));//y was 37.25

            if (worldYPosition < -50 && worldYPosition > -62) {
                intake.intake.setPower(-1);
            }

            if (worldYPosition < 30 && worldYPosition > -45) {
                intake.intake.setPower(1);
            }

            if (worldYPosition < -15 && worldYPosition > -25) {
                fourBars.setFourBarPosition(.96);
                fourBarRotator.setFourBarRotatorPosition(0.84);
            }

            if (worldYPosition < 10 && worldYPosition > -13) {
                fourBarRotator.setFourBarRotatorPosition(0.82);
                grippers.setFrontGripperPosition(0.66);
                grippers.setBackGripperPosition(0.74);
            }

            if (worldYPosition > 15) {
                intake.intake.setPower(0);
                double target = cycles == 0 ? 1100 : 1400;
                pixelLift.pControllerPixelLift.setSetPoint(target);
                fourBars.setFourBarPosition(0);
                if (pixelLift.pixelLiftPosition > 500) {
                    fourBarRotator.setFourBarRotatorPosition(.45);
                }
            }

            drive.applyMovementDirectionBased();
            pixelLift.updateLiftPosition();

            if (Movement.followCurve(points, Math.toRadians(-90)) && pixelLift.pixelLiftPosition > 700) {
                drive.stopAllMovementDirectionBased();
                grippers.frontGripper.setPosition(.16);
                grippers.backGripper.setPosition(.21);
                cycles = cycles + 1;
                if (cycles <= 2) {
                    nextStage(progStates.dropYellowPixel.ordinal());
                } else {
                    nextStage(progStates.park.ordinal());
                }
            }
        }

        if (programStage == progStates.park.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            pixelLift.updateLiftPosition();

            if (SystemClock.uptimeMillis() - stateStartTime > 450) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                //back away from backdrop
                points.add(new CurvePoint(stateStartingX, stateStartingY - 6, //changed + to -
                        0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 15,
                        Math.toRadians(60), 0.6));

                drive.applyMovementDirectionBased();

                if (Movement.followCurve(points, Math.toRadians(90))) {
                    drive.stopAllMovementDirectionBased();
                }
            } else {
                movement_y = -0.18;
                drive.applyMovementDirectionBased();

                if (SystemClock.uptimeMillis() - stateStartTime > 100) {
                    grippers.frontGripper.setPosition(.16);
                    grippers.backGripper.setPosition(.21);
                }
            }

            if (SystemClock.uptimeMillis() - stateStartTime > 750) {
                pixelTwister.setPixelTwisterPosition(.48);
                fourBarRotator.setFourBarRotatorPosition(.83);
                fourBars.setFourBarPosition(.4);

                pixelLift.pControllerPixelLift.setSetPoint(0);
            }
        }
    }
}
