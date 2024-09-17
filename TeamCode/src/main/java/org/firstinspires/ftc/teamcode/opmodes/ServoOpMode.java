package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;

@TeleOp
public class ServoOpMode extends OpMode {

    ProgrammingBot bot = new ProgrammingBot();

    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.init(hardwareMap);
    }
    public void loop(){
        bot.setServoPosition(gamepad1.left_trigger);
        telemetry.addData("Servo Position", bot.getServoPosition());
    }
}
