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
       if (gamepad1.left_trigger > .25) {
           bot.runIntake(false);
       } else if (gamepad1.right_trigger > .25) {
           bot.runIntake(true);
       } else {
           bot.stopIntake();
       }
    }

}
