package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private double kP, kI, kD, kF;
    private final double minIntegral=-1, maxIntegral = 1;

    private double lastError;
    private double integralSum;
    private ElapsedTime timer;

    public PIDFController(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.lastError = 0;
        this.integralSum = 0;
        this.timer = new ElapsedTime();

    }
    // Overloaded constructor with no Kf term
    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0); // Calls the primary constructor, setting Kf to 0
    }
    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
    }
    public double update(double targetAngle, double currentAngle){
        double error = targetAngle - currentAngle;
        double elapsedTime = timer.seconds();
        //P
        double proportional = kP* error;
        if(proportional>1){proportional=1;}
        if(proportional<-1){proportional=-1;}
        //I
        integralSum += error * elapsedTime;
        if (integralSum>maxIntegral){integralSum=maxIntegral;}
        if (integralSum<minIntegral){integralSum=minIntegral;}
        if (Math.abs(error) < 0.001) { // Example threshold
            integralSum = 0;
        }
        double integral = kI *integralSum;
        //D
        double derivative = kD * (error - lastError) / elapsedTime;
        //F
        double feedforward = kF * Math.cos(Math.toRadians(currentAngle)-(Math.PI/2));

        double output = proportional + integral + derivative + feedforward;
        lastError = error;
        timer.reset();

        output = Math.max(-1, Math.min(1, output));
        return output;
    }

    public void reset() {
        // Reset integral and last error for fresh computation
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
