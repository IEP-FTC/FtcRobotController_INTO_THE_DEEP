package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ProgrammingBot {
    public static double SPEED_ADJUSTMENT = 0.5;
    public static double TURN_ADJUSTMENT = 0.5;
    public static double TARGET_BEARING_TOLERANCE = 2.5;
    public static double MAX_POWER = .75;
    public static double MIN_POWER = 0.2;
    private DigitalChannel touchSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    public double ticksPerRotation;
    private Servo servo;
    boolean wasUp, wasDown;
    int driveModeNum = 0;

    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        leftMotor = hwMap.get(DcMotor.class, "Drive 1");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor = hwMap.get(DcMotor.class, "Drive 2");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        ticksPerRotation = leftMotor.getMotorType().getTicksPerRev()/2;//div by 2 because that's what my manual test showed me.
        servo = hwMap.get(Servo.class, "Servo");

    }

    public void setServoPosition(double position){
        servo.setPosition(position);
    }
    public double getServoPosition(){
        return servo.getPosition();
    }

    public boolean getTouchSensorState(){
        return touchSensor.getState();
    }

    public void runArcadeDrive(double forwardSpeed, double turnSpeed){
        leftMotor.setPower(forwardSpeed*SPEED_ADJUSTMENT-turnSpeed*TURN_ADJUSTMENT);
        rightMotor.setPower(forwardSpeed*SPEED_ADJUSTMENT+turnSpeed*TURN_ADJUSTMENT);
    }
    public void runTankDrive(double leftSpeed, double rightSpeed){
        leftMotor.setPower(leftSpeed*SPEED_ADJUSTMENT);
        rightMotor.setPower(rightSpeed*SPEED_ADJUSTMENT);
    }

    public void driveBot(Gamepad gamepad1, Telemetry telemetry){ //needs inputs

        if (gamepad1.dpad_up && !wasUp) {

            driveModeNum--;
            if (driveModeNum < 0) {
                driveModeNum = 1;
            }
        }
        wasUp = gamepad1.dpad_up;
        if (gamepad1.dpad_down && !wasDown){
            driveModeNum++;
            if (driveModeNum > 1) {
                driveModeNum = 0;
            }
        }
        wasDown = gamepad1.dpad_down;

        telemetry.addLine("Use Up and Down on D-pad to cycle through choices");
        switch (driveModeNum){
            case 0:
                runArcadeDrive(gamepad1.right_stick_y, gamepad1.right_stick_x);
                telemetry.addLine("Drive Mode: Arcade");
                break;
            case 1:
                runTankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
                telemetry.addLine("Drive Mode: Tank");
                break;
        }
    }

    public double getLeftMotorRotations() {
        return leftMotor.getCurrentPosition()/ticksPerRotation;
    }
    public double getRightMotorRotations() {
        return rightMotor.getCurrentPosition()/ticksPerRotation;
    }

    public void setLeftMotorPower(double leftPower) {
        leftMotor.setPower(leftPower*SPEED_ADJUSTMENT);
    }
    public void setRightMotorPower(double rightPower) {
        rightMotor.setPower(rightPower*SPEED_ADJUSTMENT);
    }
    public void driveForward(){
        leftMotor.setPower(1*SPEED_ADJUSTMENT);
        rightMotor.setPower(1*SPEED_ADJUSTMENT);
    }
    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void goToBearingAndRangeOLD(double bearing, double range){
        if (abs(bearing) > TARGET_BEARING_TOLERANCE) {
            double turn_speed = bearing/20;
            turn_speed = turn_speed*TURN_ADJUSTMENT;

            leftMotor.setPower(turn_speed);
            rightMotor.setPower(-turn_speed);
        } else if (range>10) {
            driveForward();
        } else {
            stopMotors();
        }
    }
    public void slowDown(){
        leftMotor.setPower(leftMotor.getPower()/2);
        rightMotor.setPower(rightMotor.getPower()/2);
    }

    public void adjustMotorPower(DcMotor motor, double speed){
        double motor_power = motor.getPower();
        double new_power = motor_power + speed;
        if (new_power > MAX_POWER) {
            new_power = MAX_POWER;
        } else if (new_power < MIN_POWER) {
            new_power = MIN_POWER;
        }

        motor.setPower(new_power);
    }

    public void goToBearingAndRange(double bearing, double range){
        double turn_speed;
        if (range > 15) {

            turn_speed = bearing / 40;
            if (bearing > 0) {
                adjustMotorPower(leftMotor, -turn_speed);
                adjustMotorPower(rightMotor, turn_speed);
            } else {
                adjustMotorPower(leftMotor, -turn_speed);
                adjustMotorPower(rightMotor, turn_speed);
            }

        } else {
            stopMotors();
        }
    }

}
