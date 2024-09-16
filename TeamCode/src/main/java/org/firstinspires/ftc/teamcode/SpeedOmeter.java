package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

public class SpeedOmeter {
    private static long lastUpdateStartTime = 0;
    private static double currSpeedY = 0.0;
    private static double currSpeedX = 0.0;

    //min time between updates to make sure our speed is accurate
    public static int timeBetweenUpdates = 25;
    //public static double yDistTraveled = 0;
    //public static double xDistTraveled = 0;


    public static double lastAngle = 0;

    public static double angularVelocity = 0;

    //calculates our current velocity every update
    public static void update(double xVel, double yVel, double angVel) {
        long currTime = SystemClock.uptimeMillis();

        //return if no change in telemetry
        /*if(Math.abs(yDistTraveled) < 0.000000001 && Math.abs(xDistTraveled) < 0.000000001 &&
                Math.abs(angularVelocity) < 0.000001){
            return;
        }*/

        if (currTime - lastUpdateStartTime > timeBetweenUpdates) {
            currSpeedX = -xVel;
            currSpeedY = yVel;
            angularVelocity = angVel;

            lastUpdateStartTime = currTime;
        }
    }

    /**
     * gets relative y speed in im/s
     */
    public static double getSpeedY() {
        return currSpeedY;
    }

    /**
     * gets relative x speed = im/s
     */
    public static double getSpeedX() {
        return currSpeedX;
    }

    public static double getDegPerSecond() {
        return Math.toDegrees(angularVelocity);
    }

    public static double getRadPerSecond() {
        return angularVelocity;
    }

    public static double scalePrediction = 1.0;
    //amount robot slips (cm) while going forwards 1 centimeter per second
    public static double ySlipDistanceFor1CMPS = 0.18359 * scalePrediction;//0.169;
    public static double xSlipDistanceFor1CMPS = 0.11849 * scalePrediction;//0.117;
    //radians the robot slips when going 1 radian per second
    public static double turnSlipAmountFor1RPS = 0.06485 * scalePrediction;//0.113;


    /**
     * Gives the current distance (cm) the robot would slip if power is set to 0
     */
    public static double currSlipDistanceY() {
        return SpeedOmeter.getSpeedY() * ySlipDistanceFor1CMPS;
    }

    public static double currSlipDistanceX() {
        return SpeedOmeter.getSpeedX() * xSlipDistanceFor1CMPS;
    }

    /**
     * Gives the number of radians the robot would turn if power was cut now
     */
    public static double currSlipAngle() {
        return getRadPerSecond() * turnSlipAmountFor1RPS;
    }
}

