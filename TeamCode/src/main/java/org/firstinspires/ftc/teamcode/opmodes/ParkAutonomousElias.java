package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(preselectTeleOp = "MasterMode")
public class ParkAutonomousElias extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();

    public void init() {
        mecanumDrive.init(hardwareMap);
    }

    public void loop() {
        mecanumDrive.drive(-1, 0, 0);
    }

    public void stop(){
        mecanumDrive.stop();
    }
}
