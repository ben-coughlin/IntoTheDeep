package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
@Autonomous
public class TestAuto extends AutoMaster {
    public static double SCALE_FACTOR = 0.6;



    @Override
    public void init()
    {
        super.init();
        initializeStateVariables();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


    }

    @Override
    public void start()
    {
        super.start();

    }
    @Override
    public void mainLoop()
    {
        telemetry.addData("Current State", programStage);

        if (programStage == AutoTest.progStates.trustSpikeLocationState1.ordinal()) {
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
                nextStage(AutoTest.progStates.trustSpikeLocationState2.ordinal());
            }
        }
    }
}
