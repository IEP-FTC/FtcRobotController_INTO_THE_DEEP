package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoGripperTwo {
    private CRServo servoLeft;
    private CRServo servoRight;
    private CRServo servoRotate;

    public void init(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");
        servoRotate = hardwareMap.get(CRServo.class, "servoRotate");
        servoRight.setDirection(CRServo.Direction.REVERSE);
        servoLeft.setDirection(CRServo.Direction.REVERSE);


    }
    public void moveLeftServo(double position){
        servoLeft.setPower(position);
    }

    public void moveRightServo(double position){
        servoRight.setPower(position);
    }

    public void moveRotateServo(double power){
        servoRotate.setPower(power);
    }

    public void stop(){
        servoLeft.setPower(0);
        servoRight.setPower(0);
    }
}
