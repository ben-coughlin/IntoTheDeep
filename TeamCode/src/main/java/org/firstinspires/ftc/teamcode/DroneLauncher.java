package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    private Servo droneLauncher;

    private double droneLauncherActionPosition; // variable for RR1.0 actions
    private double frontGripperPosition; // variable for regular use like on init.. humm may not need since its defined in the method?



    public DroneLauncher(HardwareMap hardwareMap) {
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");

    }

    /////////////////////////   Drone Launcher   /////////////////////////////////////
    public class DroneLauncherActionSet implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                droneLauncher.setPosition(droneLauncherActionPosition);
                initialized = true;
            }

            double pow = droneLauncher.getPosition(); // this is an example from the RR 1.0 docs I think it sends stuff to dashboard...
            packet.put("DroneLaunch", pow);
            return pow <10_000.0;
        }
    }

    public Action droneLauncherActionSet(double droneLauncherActionSetPosition) {  // this method is for use in RR trajectories
        droneLauncherActionPosition = droneLauncherActionSetPosition;
        return new DroneLauncherActionSet();
    }

    public void setDroneLauncherPosition(double droneLauncherPosition){  // this method is for basic use like init...
        droneLauncher.setPosition(droneLauncherPosition);
    }

}
