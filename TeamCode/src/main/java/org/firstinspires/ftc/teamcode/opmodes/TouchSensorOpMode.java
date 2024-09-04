package org.firstinspires.ftc.teamcode.opmodes;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor: REV touch sensor", group = "Sensor")
public class TouchSensorOpMode extends OpMode {
    ProgrammingBot bot = new ProgrammingBot();
    @Override
    public void init(){
        bot.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("Touch Sensor: ", bot.getTouchSensorState());
    }
}
