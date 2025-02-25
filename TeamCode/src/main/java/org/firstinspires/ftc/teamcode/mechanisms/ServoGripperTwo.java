package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoGripperTwo {
    private CRServo servoLeft;
    private CRServo servoRight;

    public void init(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");

    }
    public void moveLeftServo(double position){
        servoLeft.setPower(position);
    }

    public void moveRightServo(double position){
        servoRight.setPower(position);
    }

    public void stop(){
        servoLeft.setPower(0);
        servoRight.setPower(0);
    }
}
