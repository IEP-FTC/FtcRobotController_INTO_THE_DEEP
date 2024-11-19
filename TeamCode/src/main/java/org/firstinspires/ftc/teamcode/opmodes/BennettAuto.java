package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;


@Autonomous
public class BennettAuto extends OpMode {
    MecanumDrive mecanumDriveAuto = new MecanumDrive();

    @Override
    public void init() {mecanumDriveAuto.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward;
        double right;
        double rotate;

        mecanumDriveAuto.drive(1, 0, 0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        mecanumDriveAuto.drive(0, 1, 0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        mecanumDriveAuto.drive(-1, 0, 0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


    }
}