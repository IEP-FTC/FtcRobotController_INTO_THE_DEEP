package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;

@TeleOp
public class CompliantIntake extends OpMode {

    ProgrammingBot bot = new ProgrammingBot();

   @Override
    public void init () {
       bot.init(hardwareMap);
    }

    @Override
    public void loop () {
       if (gamepad1.left_trigger > .1) {
           bot.runIntake(false, gamepad1.left_trigger);
       } else if (gamepad1.right_trigger > .1) {
           bot.runIntake(true, gamepad1.right_trigger);
       } else {
           bot.stopIntake();
       }
    }

}
