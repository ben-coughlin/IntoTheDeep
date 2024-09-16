package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grippers {
    public Servo frontGripper;
    public Servo backGripper;
    private double frontGripperActionPosition; // variable for RR1.0 actions
    private double frontGripperPosition; // variable for regular use like on init.. humm may not need since its defined in the method?

    private double backGripperActionPosition; // variable for RR1.0 actions
    private double backGripperPosition; // variable for regular use like on init.. humm may not need since its defined in the method?

    public Grippers(HardwareMap hardwareMap) {
        frontGripper = hardwareMap.get(Servo.class, "gripperFront");
        backGripper = hardwareMap.get(Servo.class, "gripperBack");
    }

    /////////////////////////   Front Gripper   /////////////////////////////////////
    public class FrontGripperActionSet implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                frontGripper.setPosition(frontGripperActionPosition);
                return false;
        }
    }
    public class FrontGripperActionOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            frontGripper.setPosition(.66);
            return false;
        }
    }
    public class FrontGripperActionClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            frontGripper.setPosition(.16);
            return false;
        }
    }

    public Action frontGripperActionSet(double frontGripperActionSetPosition) {  // this method is for use in RR trajectories
        frontGripperActionPosition = frontGripperActionSetPosition;
        return new FrontGripperActionSet();
    }
    public Action frontGripperActionOpen() {  // this method is for use in RR trajectories

        return new FrontGripperActionOpen();
    }
    public Action frontGripperActionClose() {  // this method is for use in RR trajectories

        return new FrontGripperActionClose();
    }

    public void setFrontGripperPosition(double frontGripperPosition){  // this method is for basic use like init...
        frontGripper.setPosition(frontGripperPosition);
    }
/////////////////////////   Back Gripper   /////////////////////////////////////
public class BackGripperActionSet implements Action {

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

            backGripper.setPosition(backGripperActionPosition);
            return false;

    }
}

    public class BackGripperActionOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            backGripper.setPosition(.71);
            return false;

        }
    }
    public class BackGripperActionClosed implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            backGripper.setPosition(.21);
            return false;

        }
    }

    public Action backGripperActionSet(double backGripperActionSetPosition) {  // this method is for use in RR trajectories
        backGripperActionPosition = backGripperActionSetPosition;
        return new BackGripperActionSet();
    }

    public Action backGripperActionOpen() {  // this method is for use in RR trajectories

        return new FrontGripperActionOpen();
    }
    public Action backGripperActionClose() {  // this method is for use in RR trajectories

        return new BackGripperActionClosed();
    }

    public void setBackGripperPosition(double backGripperPosition){  // this method is for basic use like init...
       backGripper.setPosition(backGripperPosition);
    }

}
