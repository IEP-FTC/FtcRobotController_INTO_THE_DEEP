package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;

public class ServoIntakeOpMode extends OpMode {

    ServoIntake intake = new ServoIntake();
    double leftIntakePower;
    double rightIntakePower;

    @Override
    public void init(){
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
//        if(gamepad1.left_stick_x > 0.05){
//            leftIntakePower = gamepad1.left_stick_x;
//        }
//        intake.moveLeftServo(leftIntakePower);
//
//        if(gamepad1.right_stick_x > 0.05){
//            rightIntakePower = gamepad1.right_stick_x;
//        }
//        intake.moveRightServo(rightIntakePower);
//
//        if(gamepad1.a){
//            intake.stop();
//        }


    }
}
