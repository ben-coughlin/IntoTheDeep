package org.firstinspires.ftc.teamcode;

public class RobotPosition { // this is based on GF's code
    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    public static double moveScalingFactor = 12.56064392;
    public static double turnScalingFactor = 35.694;
    public static double auxScalingFactor = 12.48;//12.6148;
    public static double auxPredictionScalingFactor = 0.92;


    public static double wheelLeftLast = 0.0;
    public static double wheelRightLast = 0.0;
    public static double wheelAuxLast = 0.0;

    public static double worldXPosition = 0.0;
    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;

    public static double worldXPositionOld = 0.0;
    public static double worldYPositionOld = 0.0;


    public static double currPos_l = 0;
    public static double currPos_r = 0;
    public static double currPos_a = 0;


    //stuff for reading the angle in an absolute manner
    public static double wheelLeftInitialReading = 0.0;
    public static double wheelRightInitialReading = 0.0;
    public static double lastResetAngle = 0.0;//this is set when you reset the position


    //use this to get how far we have traveled in the y dimension this update
    public static double currentTravelYDistance = 0.0;

    /**
     * Makes sure an angle is in the range of -180 to 180
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2.0*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }


    public static double subtractAngles(double angle1, double angle2){
        return AngleWrap(angle1-angle2);
    }


    /**USE THIS TO SET OUR POSITION**/
    public static void setPosition(double x,double y,double angle){
        worldXPosition = x;
        worldYPosition = y;
        worldAngle_rad= angle;

        worldXPositionOld = x;
        worldYPositionOld = y;

        //remember where we were at the time of the reset
        wheelLeftInitialReading = currPos_l;
        wheelRightInitialReading = currPos_r;
        lastResetAngle = angle;
    }

    ////////////////////////////////////////////////////////////////////////////////

/* // we have two of these from some reason? I commented this one out 8/1/23
    public static float AngleWrap(float angle){
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }

 */
}
