package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutonomousOne extends OpMode {
    MecanumDrive mDrive = new MecanumDrive();
    Intake intake = new Intake();
    boolean driveForward = false;
    boolean intakeDone = false;
    public enum Steps{
        Forward,
        ReleaseSample,
        MoveToTheSide,
        Park
    }
    private Steps step = Steps.Forward;

    public void init() {
        mDrive.init(hardwareMap);
        intake.init(hardwareMap);

    }

    public void loop(){
        switch (step) {
            case Forward:

            case ReleaseSample:

            case MoveToTheSide:

            case Park:
                
        }


    }
}

