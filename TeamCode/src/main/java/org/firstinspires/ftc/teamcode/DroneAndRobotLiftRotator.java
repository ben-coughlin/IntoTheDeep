package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DroneAndRobotLiftRotator {
    public ServoImplEx droneAndRobotLiftRotator;

    private double droneAndRobotLiftRotatorActionPosition; // variable for RR1.0 actions
    private double droneAndRobotLiftRotatorPosition; // variable for regular use like on init.. humm may not need since its defined in the method?

    public DroneAndRobotLiftRotator(HardwareMap hardwareMap) {
        droneAndRobotLiftRotator = hardwareMap.get(ServoImplEx.class, "robotLiftServo");
    }

    /////////////////////////   Front Gripper   /////////////////////////////////////
    public class DroneAndRobotLiftRotatorActionSet implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                droneAndRobotLiftRotator.setPosition(droneAndRobotLiftRotatorActionPosition);
                initialized = true;
            }

            double pow = droneAndRobotLiftRotator.getPosition(); // this is an example from the RR 1.0 docs I think it sends stuff to dashboard...
            packet.put("drone and Robot Lift Rotator", pow);
            return pow <10_000.0;
        }
    }

    public Action droneAndRobotLiftActionSet(double droneAndRobotLiftActionSetPosition) {  // this method is for use in RR trajectories
        droneAndRobotLiftRotatorActionPosition = droneAndRobotLiftActionSetPosition;
        return new DroneAndRobotLiftRotatorActionSet();
    }

    public void setDroneAndRobotLiftRotatorPosition(double droneAndRobotLiftRotatorPosition){  // this method is for basic use like init...
        droneAndRobotLiftRotator.setPosition(droneAndRobotLiftRotatorPosition);
    }


}
