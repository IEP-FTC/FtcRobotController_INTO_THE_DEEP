package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;

@TeleOp
public class Arm extends OpMode {
    Intake intake = new Intake();
    ArmPivot armPivot = new ArmPivot();
    Slide slide = new Slide();
    boolean toggleState = false;
    boolean aPressed = false;

    @Override
    public void init() {
        intake.init(hardwareMap);
        armPivot.init(hardwareMap);
        slide.init(hardwareMap);

    }
    @Override
    public void loop() {

        //Intake Control B in X out
        if (gamepad1.b) {
            intake.runIntake(true, 1);
        } else if (gamepad1.x) {
            intake.runIntake(false, 1);
        }//TODO Add stop intake

        //Slide R Trigger extend L trigger contract
        if (gamepad1.left_trigger > .1) {
            slide.runSlide(false, gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > .1) {
            slide.runSlide(true, gamepad1.right_trigger);
        } else {
            slide.runSlide(true,0);
        }//TODO add full extension/retract on bumpers

        //ArmPivot A 110 degrees toggle
        if (gamepad1.a && !aPressed) {
            toggleState = !toggleState; // Toggle state
            aPressed = true;           // Set flag to prevent retrigger
        } else if (!gamepad1.a) {
            aPressed = false;          // Reset flag when button is released
        }

        if (toggleState) {
            armPivot.moveToPosition(110);//TODO adjust angle to correct (<110)

        } else {
            armPivot.moveToPosition(0);
        }

    }
}

