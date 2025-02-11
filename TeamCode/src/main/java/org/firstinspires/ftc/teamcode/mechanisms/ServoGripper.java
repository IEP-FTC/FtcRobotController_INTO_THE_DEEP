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
    }

    public void open(){

        intakeServoLeft.setPosition(0);
        intakeServoRight.setPosition(1);
    }
    public void close(){

        intakeServoLeft.setPosition(1);
        intakeServoRight.setPosition(.3);
    }


    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Left Servo Position:", intakeServoLeft.getPosition());
        telemetry.addData("Right Servo Position:", intakeServoRight.getPosition());
    }

}

