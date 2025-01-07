package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.ArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;

@TeleOp
public class MasterMode extends OpMode {
    private DcMotor slideMotor;
    Intake intake = new Intake();
    ArmPivot armPivot = new ArmPivot();
    Slide slide = new Slide();
    MecanumDrive mecanumDrive = new MecanumDrive();
    boolean armState = false;
    boolean aPressed = false;


    @Override
    public void init() {
        intake.init(hardwareMap);
        armPivot.init(hardwareMap);
        slide.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.b) {
            intake.runIntake(true);
        } else if (gamepad1.x) {
            intake.runIntake(false);
        } else {
            intake.stopIntake();
        }

        if (gamepad1.right_trigger >= 0.1) {
            slide.runSlide(true, gamepad1.right_trigger);
        } else if (gamepad1.left_trigger >= 0.1) {
            slide.runSlide(true, -gamepad1.left_trigger);
        }

        if (gamepad1.a && !aPressed) {
            armState = !armState;
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        if (armState) {
            armPivot.moveToPosition(110);
        } else {
            armPivot.moveToPosition(0);
        }
        mecanumDrive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

    }
}