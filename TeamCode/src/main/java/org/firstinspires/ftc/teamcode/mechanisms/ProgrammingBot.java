package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ProgrammingBot {
    public static double SPEED_ADJUSTMENT = 0.25;
    public static double TURN_ADJUSTMENT = 0.5;
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
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor = hwMap.get(DcMotor.class, "Drive 2");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void driveBot(Gamepad gamepad1){ //needs inputs

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
}
