package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.mechanisms.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Drive")
public class DriveOpMode extends OpMode {
    ProgrammingBot bot = new ProgrammingBot();

    boolean wasUp, wasDown;
    int driveModeNum = 0;

    @Override
    public void init(){
        bot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up && !wasUp){
            driveModeNum--;
            if (driveModeNum < 0) {
                driveModeNum = 1;
            }
        }
        wasUp = gamepad1.dpad_up;
        if (gamepad1.dpad_down && !wasDown){
            driveModeNum++;
            if (driveModeNum > 1) {
                driveModeNum = 0;
            }
        }
        wasDown = gamepad1.dpad_down;

        telemetry.addLine("Use Up and Down on D-pad to cycle through choices");
        switch (driveModeNum){
            case 0:
                bot.runArcadeDrive(gamepad1.right_stick_y, gamepad1.right_stick_x);
                telemetry.addLine("Drive Mode: Arcade");
                break;
            case 1:
                bot.runTankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
                telemetry.addLine("Drive Mode: Tank");
                break;
        }

    }


}
