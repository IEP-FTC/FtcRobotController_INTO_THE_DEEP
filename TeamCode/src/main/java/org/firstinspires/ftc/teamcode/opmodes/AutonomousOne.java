package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

    @Autonomous
    public class AutonomousOne extends OpMode {
        MecanumDrive mDriveVOne = new MecanumDrive();
        Intake intake = new Intake();
        boolean driveForward = false;
        boolean intakeDone = false;

        public void init() {
            mDriveVOne.init(hardwareMap);
            intake.init(hardwareMap);

        }

    public void loop(){
        if(!driveForward) {
            mDriveVOne.drive(0, -1, 0);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            mDriveVOne.drive(1, 0, 0);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }


            //intake code here
            if (!intakeDone) {
                intake.runIntake(1);
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                intake.stopIntake();
                intakeDone = true;
            } else {
                intake.stopIntake();
            }
            // drive backwards, what about other team
            // go to ascent zone

            if (intakeDone) {
                mDriveVOne.drive(0, -1, 0);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mDriveVOne.drive(-1,0,0);

            } else {
                mDriveVOne.stop();
            }
            driveForward = true;
        } else {
            mDriveVOne.stop();
            intake.stopIntake();
        }

    }
}