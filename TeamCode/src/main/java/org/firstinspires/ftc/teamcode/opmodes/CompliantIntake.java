package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;
import org.firstinspires.ftc.teamcode.mechanisms.ServoGripper;
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;

@TeleOp
public class CompliantIntake extends OpMode {

    ServoGripper intake = new ServoGripper();
    double leftServoPosition;
    double rightServoPosition;

   @Override
    public void init () {
       intake.init(hardwareMap);
    }

    @Override
    public void loop () {
       if(gamepad1.left_stick_x > 0.05){
           leftServoPosition += gamepad1.left_stick_x/50;
       }
       intake.moveLeftServo(leftServoPosition);

       if(gamepad1.right_stick_x > 0.05){
            rightServoPosition += gamepad1.left_stick_x/50;
       }
       intake.moveRightServo(rightServoPosition);

       if(leftServoPosition > 1){
           leftServoPosition = 1;
       }

        if(rightServoPosition > 1){
            rightServoPosition = 1;
        }


    }



}
