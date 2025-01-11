package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

public class ParkAutonomousElias {
    MecanumDrive mecanumDrive = new MecanumDrive();

    public void init() {
        mecanumDrive.init(hardwareMap);
    }

    public void loop() {
        mecanumDrive.drive(1, 0, 0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        mecanumDrive.drive(0, 0, 0);
    }
}
