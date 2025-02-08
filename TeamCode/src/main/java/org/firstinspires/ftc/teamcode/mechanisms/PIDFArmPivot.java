package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libraries.PIDFController;

@Config
public class PIDFArmPivot {
    private DcMotor pivotLeft, pivotRight;
    private PIDFController pidfController;
    public double currentAngle=65,targetA=65,power;
    public static double armRestAngle=65;
    public static double kP=0.1, kI=0.01, kD=0.003, kF=0.15;

    public void init(HardwareMap HardMap) {
        pivotLeft = HardMap.get(DcMotor.class, "pivotLeft");
        pivotRight = HardMap.get(DcMotor.class, "pivotRight");

        pivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotLeft.setDirection(DcMotor.Direction.REVERSE);

        pivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the PIDF Controller (adjust gains as needed)
        pidfController = new PIDFController(kP, kI, kD, kF);

    }
    public double ticksToArmDegrees(double ticks) {
        return ticks * (360.0 / 288) / (125.0/30);//125 = big gear, 30 = small gear, 360/288 = tick ratio
    }

    public void moveToAngle(double targetAngle) {
        targetA = targetAngle;
        // Get the current position of the arm
        currentAngle = ticksToArmDegrees(pivotLeft.getCurrentPosition())+armRestAngle;
        //For tuning purposes, update coeficients
        pidfController.setCoefficients(kP, kI, kD, kF);
        // Use PIDF to calculate power
        power = pidfController.update(targetA, currentAngle);


        // Set power to motors
        pivotLeft.setPower(power);
        pivotRight.setPower(power);

    }

    public double getTargetAngle(){
        return targetA;
    }
    public double getCurrentAngle(){
        return currentAngle;
    }

    public void holdPivot() {
        pivotLeft.setPower(0);
        pivotRight.setPower(0);
        pidfController.reset();
    }


    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Arm currentAngle", currentAngle);
        telemetry.addData("Arm targetAngle",targetA);
        telemetry.addData("Arm PIDF power",power*100);
    }
    public void resetEncoder(){
        pivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotLeft.setTargetPosition(0);
        pivotRight.setTargetPosition(0);
    }
    public void resetArmPosition(){
        pivotLeft.setPower(-0.1);
        pivotRight.setPower(-0.1);
    }

}
