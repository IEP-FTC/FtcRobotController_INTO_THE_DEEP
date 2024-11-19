package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;


@TeleOp()
public class SimpleMecanumDriveOpMode extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();

    boolean wasUp, wasDown;
    int driveModeNum = 0;

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    }

    @Override
    public void loop(){
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

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
                mecanumDrive.drive(forward, right, rotate);
                telemetry.addLine("Drive Mode: Normal");
                break;
            case 1:
                mecanumDrive.driveGyroCorrected(forward, right, rotate);
                telemetry.addLine("Drive Mode: Gyro Corrected");
                break;
        }

    }
}
