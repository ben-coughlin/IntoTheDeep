package org.firstinspires.ftc.teamcode;

public class PController {
    public double setPoint = 0, minInput = 0, maxInput = 0, minOutput = 0;
    public double maxOutput = 0, thresholdPercent = 0;
    private double currentError = 0;

    private double Kp;
    public PController(double Kp) { this.Kp = Kp; }

    public void setThresholdValue (double thresholdPercent) {
        this.thresholdPercent = thresholdPercent; }
    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }
    public void setInputRange(double minInput, double maxInput) {
        this.minInput = Math.abs(minInput);
        this.maxInput = Math.abs(maxInput);
    }
    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }
    public double getComputedOutput(double input) {
        // get the error as absolute value - the sign of error is immaterial since
        // that gets taken care of where the P Controller value is used
        currentError = Math.abs(Math.abs(setPoint) - Math.abs(input));
        double computedOutput =  currentError * Kp * (maxOutput - minOutput);
        if (computedOutput > (maxOutput-minOutput)) {
            computedOutput = (maxOutput-minOutput);
        }
        return computedOutput;
    }
    public boolean hasPControllerReachedTarget() {
        double percentDifferenceFromTarget =
                (currentError /(maxInput - minInput))*100;
        if(percentDifferenceFromTarget < thresholdPercent) {
            return true;
        }
        return false;
    }
}
