package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ServoGripperTwo;
@TeleOp

public class ServoIntakeTwo extends OpMode {

    ServoGripperTwo gripper = new ServoGripperTwo();
    double leftIntakePower;
    double rightIntakePower;
    double rotateIntakePower;

    @Override
    public void init() {
        gripper.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_x > 0.1){
           leftIntakePower = gamepad1.left_stick_x;
       } else if (gamepad1.left_stick_x < -0.1){
            leftIntakePower = gamepad1.left_stick_x;
        } else {
            leftIntakePower = 0;
        }
        gripper.moveLeftServo(leftIntakePower);

       if(gamepad1.right_stick_x < -0.1){
           rightIntakePower = gamepad1.right_stick_x;
        } else if (gamepad1.right_stick_x > 0.1){
           rightIntakePower = gamepad1.right_stick_x;
       } else {
           rightIntakePower = 0;
       }
       gripper.moveRightServo(rightIntakePower);

        if(gamepad1.a){
            gripper.stop();
        }

        if(gamepad1.dpad_down){
            rotateIntakePower = -0.5;
        }
        if(gamepad1.dpad_up){
            rotateIntakePower = 0.5;
        }
        gripper.moveRotateServo(rotateIntakePower);
        }
}
