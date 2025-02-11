package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;

@TeleOp
public class CompliantIntake extends OpMode {

    ServoIntake intake = new ServoIntake();

   @Override
    public void init () {
       intake.init(hardwareMap);
    }

    @Override
    public void loop () {
       if (gamepad1.a) {
           intake.runIntake(false);
       } else if (gamepad1.b) {
           intake.runIntake(true);
       } else {
           intake.stopIntake();
       }

       telemetry.addData("Gamepad1 A",gamepad1.a);
       telemetry.addData("Gamepad1 B",gamepad1.b);
       intake.addTelemetry(telemetry);
       telemetry.update();
    }



}
