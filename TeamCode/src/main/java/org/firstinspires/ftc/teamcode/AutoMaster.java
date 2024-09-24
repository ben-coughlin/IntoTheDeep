package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public abstract class AutoMaster extends OpMode {
    MecanumDrive drive = null;
    PixelLift pixelLift = null;
    public DcMotorEx robotLift = null;

    FourBars fourBars = null;
    Intake intake = null;
    Grippers grippers = null;
    FourBarRotator fourBarRotator = null;
    PixelTwister pixelTwister = null;
    DroneAndRobotLiftRotator droneAndRobotLiftRotator = null;
    DroneLauncher droneLauncher = null;

    IntakeServo intakeServo = null;

    DigitalChannel limitSwitch = null;

    LimeLight limeLight = null;

    //public DcMotorEx lift = null;

    //variables for pixel lift
    boolean rrLoopOn = true;
    int liftPosition = 0;
    int liftHome = 0;
    double liftMotorMax = 340 * 0.8;     //NeveRest 20 80% max rev/min
    double liftTicksPerRev = 537.6; //NeveRest 20 encoder spec  ticks per rev
    double liftVelocity = liftMotorMax * liftTicksPerRev / 60;
    double liftMaxTicks = 2370;
    // for lift pController input
    double minPowerLift = .001;
    double maxPowerLift = 0.5;

    ///variables for Robot lift
    int robotLiftPosition = 0;
    int robotLiftHome = 0;
    double robotLiftMotorMax = 340 * 0.85;     //NeveRest 20 80% max rev/min
    double robotLiftTicksPerRev = 537.6; //NeveRest 20 encoder spec  ticks per rev
    double robotLiftVelocity = liftMotorMax * liftTicksPerRev / 60;
    double robotLiftMaxTicks = 15000;
    double minPowerRobotLift = .001;
    double maxPowerRobotLift = 0.98;

    public int spikeLocation;

    AnalogInput rightDistance;
    AnalogInput leftDistance;


//clocks

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();

    //Lift P controllers
    PController pControllerRobotLift = new PController(0.002); // this is for lift left, change Kp to calibrate
    PController pControllerLift = new PController(0.002); // this is for lift right, change Kp to calibrate

    Servo pawright;

    // Vision for Tensor

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231227_193805.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "TPB",
            "TPR",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //public TfodProcessor tfod;
    public Limelight3A limelight;
    /**
     * The variable to store our instance of the vision portal.
     */
    //private VisionPortal visionPortal;


    //////// STATE MACHINE STUFF BELOW DO NOT TOUCH ////////
    public boolean stageFinished = true;
    public long stateStartTime = 0;

    public long programStartTime = 0;//time the program starts
    public static int programStage = 0;

    /**
     * STATE STARTING VARIABLES
     */
    public double stateStartingX = 0;
    public double stateStartingY = 0;
    public double stateStartingAngle_rad = 0;

    private final boolean DEBUGGING = false;

    private boolean inDebugState = false;

    //holds the stage we are going to next
    int nextStage = 0;

    public void nextStage(int ordinal) {
        nextStage = ordinal;
        //waits for a if on debug mode
        if (!DEBUGGING) {
            incrementStage();
            inDebugState = false;
        }

        //go into debug mode
        if (DEBUGGING) {
            inDebugState = true;
        }
    }

    /**
     * Increments the programStage
     */
    public void nextStage() {
        nextStage(programStage + 1);

    }

    private void incrementStage() {
        programStage = nextStage;
        stageFinished = true;
    }
    ///////////////////////////////

    DigitalChannel frontPixelReceiver;
    DigitalChannel backPixelReceiver;

    public ArrayList<CurvePoint> mirrorPoints(ArrayList<CurvePoint> points) {
        ArrayList<CurvePoint> newPoints = new ArrayList<>();
        for (CurvePoint point : points) {
            newPoints.add(new CurvePoint(-point.x, point.y, point.moveSpeed, point.turnSpeed, point.followDistance, point.pointLength, point.slowDownTurnRadians, point.slowDownTurnRadians));
        }
        return newPoints;
    }

    //Vision vision;

    @Override
    public void init() {
        //initTfod();


        pControllerRobotLift.setInputRange(0, robotLiftMaxTicks);
        pControllerRobotLift.setSetPoint(0);
        pControllerRobotLift.setOutputRange(minPowerRobotLift, maxPowerRobotLift);
        pControllerRobotLift.setThresholdValue(5);

    }

    public double mmToIn(double in) {
        return in / 25.4;
    }

    public double getRightDistanceIn() {
        return mmToIn((rightDistance.getVoltage() / (3.3 / 1024)) * 6 - 300) + 4;
    }

    public double getLeftDistanceIn() {
        return mmToIn((leftDistance.getVoltage() / (3.3 / 1024)) * 6 - 300) + 4;
    }

    private int pixelData = 0;
    private ArrayList<Boolean> count = new ArrayList<>();

    public boolean gatherPixelData() {
        if (pixelData == 5) {
            int countOfTrue = 0;
            for (Boolean val : count) {
                if (val) {
                    countOfTrue++;
                }
            }
            count.clear();

            return countOfTrue > 3;
        }

        boolean frontSeen = !frontPixelReceiver.getState();
        boolean backSeen = !backPixelReceiver.getState();

        count.add(frontSeen && backSeen);

        pixelData += 1;

        return false;
    }

    private int position = 1;

    public int pixelsInConveyor() {
        boolean hasFront = !frontPixelReceiver.getState();
        boolean hasBack = !backPixelReceiver.getState();

        if (hasFront && hasBack) {
            return 2;
        } else if (hasFront || hasBack) {
            return 1;
        } else {
            return 0;
        }
    }

    @Override
    public void init_loop() {
        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        if (ButtonPress.isGamepad1_a_pressed()) {
            position += 1;
        }

        if (ButtonPress.isGamepad1_b_pressed()) {
            position -= 1;
        }

        if (position > 2) {
            position = 2;
        } else if (position < 1) {
            position = 1;
        }

//        if (ButtonPress.isGamepad1_dpad_down_pressed()) {
//            if (position == 1) {
//                Vision.setSample1Y(Vision.getSample1Y() - 10);
//            } else {
//                Vision.setSample2Y(Vision.getSample2Y() - 10);
//            }
//        }
//
//        if (ButtonPress.isGamepad1_dpad_up_pressed()) {
//            if (position == 1) {
//                Vision.setSample1Y(Vision.getSample1Y() + 10);
//            } else if (position == 2) {
//                Vision.setSample2Y(Vision.getSample2Y() + 10);
//            } else {
//                Vision.setSample3Y(Vision.getSample3Y() + 10);
//            }
//        }
//
//        if (ButtonPress.isGamepad1_dpad_left_pressed()) {
//            if (position == 1) {
//                Vision.setSample1X(Vision.getSample1X() - 10);
//            } else if (position == 2) {
//                Vision.setSample2X(Vision.getSample2X() - 10);
//            } else {
//                Vision.setSample3X(Vision.getSample3X() - 10);
//            }
//        }
//
//        if (ButtonPress.isGamepad1_dpad_right_pressed()) {
//            if (position == 1) {
//                Vision.setSample1X(Vision.getSample1X() + 10);
//            } else if (position == 2) {
//                Vision.setSample2X(Vision.getSample2X() + 10);
//            } else {
//                Vision.setSample3X(Vision.getSample3X() + 10);
//            }
//        }
//
//        telemetry.addData("Position to change", position);
//        telemetry.addData("Position 1 Box X", Vision.getSample1X());
//        telemetry.addData("Position 1 Box Y", Vision.getSample1Y());
//        telemetry.addData("Position 2 Box X", Vision.getSample2X());
//        telemetry.addData("Position 2 Box Y", Vision.getSample2Y());
//
//        telemetry.addData("Error Left", Vision.errorLeft);
//        telemetry.addData("Error Right", Vision.errorRight);
//        telemetry.addData("H", Vision.h);
//        telemetry.addData("S", Vision.s);
//        telemetry.addData("V", Vision.v);
//
//        telemetry.addData("We Are on Right", Vision.weAreOnRightSide);
//
//        telemetry.addData("Is Red", Vision.isRed);
//
//        if (Vision.getCubeLocation() == 0) {
//            telemetry.addData("Current Team Maker Position", "Left");
//        } else if (Vision.getCubeLocation() == 1) {
//            telemetry.addData("Current Team Maker Position", "Center");
//        } else if (Vision.getCubeLocation() == 2) {
//            telemetry.addData("Current Team Maker Position", "Right");
//        }
    }

    @Override
    public void start() {
        programStage = 0;

        //vision.stopStreamingVision();
        //robotLiftAlignServo.setPosition(robotLiftRotatorPosition);
    }

    @Override
    public void loop() {
        double startLoopTime = SystemClock.uptimeMillis();
        PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();

        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        worldXPosition = drive.pose.position.x;
        worldYPosition = drive.pose.position.y;
        worldAngle_rad = drive.pose.heading.toDouble();

        // DO NOT CHANGE THIS LINE
        SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);

        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        mainAutoLoop();

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
    }

    public void initializeStateVariables() {
        stateStartingX = worldXPosition;
        stateStartingY = worldYPosition;
        stateStartingAngle_rad = worldAngle_rad;
        stateStartTime = SystemClock.uptimeMillis();
        Movement.initCurve();
        stageFinished = false;
    }

    private void mainAutoLoop() {
        if (inDebugState) {
            //drive.stopAllMovementDirectionBased(); - THIS METHOD IS DEPRECATED!!!!!!!!!!
            // ControlMovement(); CHANGE THIS

            telemetry.addLine("in debug state");
            if (gamepad1.a) {
                incrementStage();
                inDebugState = false;
            }
        } else {
            mainLoop();
        }
    }

    public abstract void mainLoop();
//commented out old vision init because it was deprecated by limelight (9/16)
//    private void initVision() {
//
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                .setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                .setModelLabels(LABELS)
//                .setIsModelTensorFlow2(true)
//                .setIsModelQuantized(true)
//                .setModelInputSize(300)
//                .setModelAspectRatio(16.0 / 9.0)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        tfod.setMinResultConfidence(0.70f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        visionPortal.setProcessorEnabled(tfod, true);
//
//    }
}
