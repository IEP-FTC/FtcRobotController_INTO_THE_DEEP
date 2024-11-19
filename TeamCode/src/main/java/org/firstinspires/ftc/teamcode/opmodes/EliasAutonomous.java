package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "EliasAutonomous")
public class EliasAutonomous extends OpMode {

    MecanumDrive mecanumDrive = new MecanumDrive();

    private boolean isGoing = true;

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (isGoing) {
            mecanumDrive.drive(0.5, 0.0, 0.0);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mecanumDrive.drive(0.0, 0.5, 0.0);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mecanumDrive.drive(-0.5, 0.0, 0.0);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            isGoing = false;
        }
    }
}