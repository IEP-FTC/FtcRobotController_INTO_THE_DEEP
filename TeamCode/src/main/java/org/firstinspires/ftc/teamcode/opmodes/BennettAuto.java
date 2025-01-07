package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Disabled
@Autonomous
public class BennettAuto extends OpMode {
    MecanumDrive mecanumDriveAuto = new MecanumDrive();
    boolean doneOnce = false;

    @Override
    public void init() {mecanumDriveAuto.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (!doneOnce) {
            mecanumDriveAuto.drive(1, 0, 0);
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mecanumDriveAuto.drive(0, 1, 0);
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mecanumDriveAuto.drive(-1, 0, 0);
            try {
                Thread.sleep(750);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            doneOnce = true;
        }

        mecanumDriveAuto.drive(0,0,0);


    }

}