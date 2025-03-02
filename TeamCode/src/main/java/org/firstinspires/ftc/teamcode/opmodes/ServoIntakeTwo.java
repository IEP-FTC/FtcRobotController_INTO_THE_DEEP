package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.ServoGripperTwo;

public class ServoIntakeTwo extends OpMode {

    ServoGripperTwo gripper = new ServoGripperTwo();
    double leftIntakePower;
    double rightIntakePower;
    @Override
    public void init() {
        gripper.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_x > 0.05){
           leftIntakePower = gamepad1.left_stick_x;
       }
        gripper.moveLeftServo(leftIntakePower);

       if(gamepad1.right_stick_x > 0.05){
           rightIntakePower = gamepad1.right_stick_x;
        }
       gripper.moveRightServo(rightIntakePower);

        if(gamepad1.a){
            gripper.stop();
        }

        }
}
