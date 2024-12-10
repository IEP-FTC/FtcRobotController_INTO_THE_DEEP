package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;


@TeleOp()
public class SimpleMecanumDriveOpMode extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    }

    @Override
    public void loop(){
        double forward = gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        mecanumDrive.drive(forward, right, rotate);
    }
}
