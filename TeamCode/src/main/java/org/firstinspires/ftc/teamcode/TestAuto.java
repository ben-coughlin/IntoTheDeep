package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
@Autonomous
public class TestAuto extends AutoMaster {
    public static double SCALE_FACTOR = 1.0;



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



        ArrayList<CurvePoint> points = new ArrayList<>();
        points.add(new CurvePoint(stateStartingX, stateStartingY,
                0, 0, 0, 0, 0, 0));

        points.add(new CurvePoint(20, 0,
                0.3 * SCALE_FACTOR, 0.5 * SCALE_FACTOR, 10, 10,
                Math.toRadians(60), 0.6));

        drive.applyMovementDirectionBased();


        if (Movement.followCurve(points, Math.toRadians(-90), 4)) {
            drive.stopAllMovementDirectionBased();
            nextStage();
        }
        //example curvepoint
//        points.add(new CurvePoint(-19, -6.5,
//                0.4 * SCALE_FACTOR, 0.5 * SCALE_FACTOR, 10, 10,
//                Math.toRadians(60), 0.6));
    }
}
