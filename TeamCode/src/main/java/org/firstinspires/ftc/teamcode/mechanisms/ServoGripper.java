package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoGripper {

    private Servo intakeServoLeft;
    private Servo intakeServoRight;

    public void init(HardwareMap hardwareMap) {
        intakeServoLeft = hardwareMap.get(Servo.class, "servoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "servoRight");
        //intakeServoRight.setDirection(Servo.Direction.REVERSE);
    }

    public void open(){
        //use the scaleRange to set the range to different angles during TeleOp so as to not be annoying
        //move 90 translates to what in robot language?
        //if servo range is 180, then 90 degrees = 0.5 in setPosition
        //if servo range is 360, then 90 degrees = 0.25 in setPosition

        intakeServoLeft.setPosition(0);
        intakeServoRight.setPosition(1);
    }
    public void close(){
        intakeServoLeft.setPosition(1);
        intakeServoRight.setPosition(.3);
    }

    public void moveLeftServo(double position){
        intakeServoLeft.setPosition(position);
    }

    public void moveRightServo(double position){
        intakeServoRight.setPosition(position);

    }


    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Left Servo Position:", intakeServoLeft.getPosition());
        telemetry.addData("Right Servo Position:", intakeServoRight.getPosition());
    }

}

