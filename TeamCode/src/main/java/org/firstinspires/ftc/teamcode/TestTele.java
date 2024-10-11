/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
//import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
//import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;
import java.lang.Math;


/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TestTele", group="teleop")
public class TestTele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    Intake intake = null;
    Lift lift = null;
    Pivot pivot = null;
    GoBildaPinpointDriver odo;
    IMU imu;


    ElapsedTime waitTimer1 = new ElapsedTime();

    enum GrabSample
    {
        OPEN_JAW,
        MOVE_WRIST,
        INTAKE_ON,
    }

    GrabSample grabSample = GrabSample.OPEN_JAW;

    //arcade vars
    double denominator = 0;
    double rotX = 0;
    double rotY = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;

    double imuYawRadians = 0;
    double imuYawDegrees = 0;
    double imuYawAutoCorrection = 90;

    double leftStickX = 0;
    double leftStickY = 0;
    double rightStickX = 0;

    double twistPos = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        imu = hardwareMap.get(IMU.class, "imu");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        twistPos = intake.twistIntake.getPosition();

        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        orientation.getYaw(AngleUnit.RADIANS);

        imuYawRadians = orientation.getYaw(AngleUnit.RADIANS) - (Math.toRadians(imuYawAutoCorrection));

        imuYawDegrees = (imuYawRadians * 57.2957795) - (Math.toRadians(imuYawAutoCorrection));

        //gamepad reads
        leftStickX = gamepad1.left_stick_x;
        leftStickY = -gamepad1.left_stick_y;
        double oldTime = 0;

        odo.update();
        //// DRIVE ************************************************************************
        //Finds the hypotenuse of the triangle created by the two joystick values. Used to find the absolute direction to go in.


        // this is gm0 arcade code added on 11/21/23 above is what we used before
        rotX = leftStickX * Math.cos(-imuYawRadians) - leftStickY * Math.sin(-imuYawRadians);
        rotY = leftStickX * Math.sin(-imuYawRadians) + leftStickY * Math.cos(-imuYawRadians);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);
        v2 = (rotY + rotX + rightStickX) / denominator; // frontLeft
        v4 = (rotY - rotX + rightStickX) / denominator; //backLeft
        v1 = (rotY - rotX - rightStickX) / denominator; //frontRight
        v3 = (rotY + rotX - rightStickX) / denominator; // backRight




        double movement_y = -gamepad1.left_stick_y;
        double movement_x = gamepad1.left_stick_x;
        double movement_turn = -gamepad1.right_stick_x;

        double fl_power_raw = movement_y - movement_turn + movement_x * 1.5;
        double bl_power_raw = movement_y - movement_turn - movement_x * 1.5;
        double br_power_raw = movement_y + movement_turn + movement_x * 1.5;
        double frPowerRaw = movement_y + movement_turn - movement_x * 1.5;

        //find the maximum of the powers
        double maxRawPower = Math.abs(fl_power_raw);

        if (Math.abs(bl_power_raw) > maxRawPower) {
            maxRawPower = Math.abs(bl_power_raw);
        }
        if (Math.abs(br_power_raw) > maxRawPower) {
            maxRawPower = Math.abs(br_power_raw);
        }
        if ((frPowerRaw) > maxRawPower) {
            maxRawPower = Math.abs(frPowerRaw);
        }

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if (maxRawPower > 1.0) {
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0 / maxRawPower;

            fl_power_raw *= scaleDownAmount;
            bl_power_raw *= scaleDownAmount;
            br_power_raw *= scaleDownAmount;
            frPowerRaw *= scaleDownAmount;

            //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
            leftFront.setPower(fl_power_raw);
            leftBack.setPower(bl_power_raw);
            rightBack.setPower(br_power_raw);
            rightFront.setPower(frPowerRaw);


            switch (grabSample) {
                case OPEN_JAW: {
                    if (ButtonPress.isGamepad1_right_bumper_pressed()) {
                        //rotate pivot into pickup position
                        intake.jawServo.setPosition(Constants.JAW_OPEN);

                        waitTimer1.reset();
                    }

                        grabSample = GrabSample.MOVE_WRIST;

                }
                case MOVE_WRIST: {
                    //extend lift into pickup position
                    if(ButtonPress.isGamepad1_dpad_left_pressed())
                    {
                        intake.lateralIntakeServo.setPosition(Constants.LATERAL_INTAKE_LEFT);
                    }
                    else if(ButtonPress.isGamepad1_dpad_right_pressed())
                    {
                        intake.lateralIntakeServo.setPosition(Constants.LATERAL_INTAKE_RIGHT);
                    }



                    grabSample = GrabSample.MOVE_WRIST;
                }
                case INTAKE_ON: {
                    intake.setIntakeSpeed(1);

                    if (waitTimer1.seconds() > 1) {
                        intake.setIntakeSpeed(0);
                    }

                    grabSample = GrabSample.OPEN_JAW;
                }
                break;
            }


            //this moves the intake to the right or left when the triggers are pressed
            while(gamepad1.right_trigger > 0)
            {
                twistPos += 0.05;
            }
            while (gamepad1.left_trigger > 0 )
            {
                twistPos -= 0.05;
            }

            twistPos = Range.clip(twistPos, 0, 1);

            intake.twistIntake.setPosition(twistPos);



            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

         /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.addData("state:", grabSample);
            telemetry.addData("twist position", twistPos);
            telemetry.update();


        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    }
