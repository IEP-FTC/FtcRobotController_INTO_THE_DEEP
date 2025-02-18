package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoIntake {

   private CRServo intakeServoLeft;
   private CRServo intakeServoRight;

    public void init(HardwareMap hardwareMap) {

        intakeServoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "servoRight");

    }

    public void moveLeftServo(double powerLeft){
        intakeServoLeft.setPower(powerLeft);
    }

    public void moveRightServo(double powerRight){
        intakeServoRight.setPower(powerRight);
    }
    public void stop(){
        intakeServoLeft.setPower(0);
        intakeServoLeft.setPower(0);
    }
}


