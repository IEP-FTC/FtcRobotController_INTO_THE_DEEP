package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;

@TeleOp(name = "Drive")
public class DriveOpMode extends OpMode {
    ProgrammingBot bot = new ProgrammingBot();


    @Override
    public void init(){
        bot.init(hardwareMap);
    }

    @Override
    public void loop(){
        bot.runArcadeDrive(gamepad1.right_stick_y, gamepad1.right_stick_x);
        //bot.setLeftMotorSpeed(gamepad1.left_stick_y);
        //bot.setRightMotorSpeed(gamepad1.right_stick_y);
    }


}
