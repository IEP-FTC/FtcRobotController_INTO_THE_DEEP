package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@TeleOp
public class MasterMode extends OpMode {
    public static int PIVOTANGLE = 78;
    Intake intake = new Intake();
    ArmPivot armPivot = new ArmPivot();
    Slide slide = new Slide();
    boolean toggleState = false;
    boolean aPressed = false;
    MecanumDrive mecanumDrive = new MecanumDrive();

    double joystickArmPosition = 0;

    @Override
    public void init() {
        intake.init(hardwareMap);
        armPivot.init(hardwareMap);
        slide.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
    }
    @Override
    public void loop() {

        //Intake Control B in X out
        if (gamepad1.b) {
            intake.runIntake(true);
        } else if (gamepad1.x) {
            intake.runIntake(false);
        } else {
            intake.stopIntake();
        }

        //Slide R Trigger extend L trigger contract
        if (gamepad1.left_trigger > .1) {
            slide.runSlide(false, gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > .1) {
            slide.runSlide(true, gamepad1.right_trigger);
        } else {
            slide.stopSlide();
        }//TODO add full extension/retract on bumpers

        //ArmPivot A 110 degrees toggle
        if (gamepad1.a && !aPressed) {
            toggleState = !toggleState; // Toggle state
            aPressed = true;           // Set flag to prevent retrigger
        } else if (!gamepad1.a) {
            aPressed = false;          // Reset flag when button is released
        }

//        if (toggleState) {
//            armPivot.moveToPosition(PIVOTANGLE);//TODO adjust angle to correct (<110)
//
//        } else {
//            armPivot.moveToPosition(0);
//        }

       if(gamepad1.a){
           armPivot.moveToPosition(PIVOTANGLE);
           joystickArmPosition = PIVOTANGLE;
       }
       if(gamepad1.y){
           armPivot.moveToPosition(0);
           joystickArmPosition = 0;
       }

       if(abs(gamepad1.left_stick_y)>0.1){
            joystickArmPosition += -gamepad1.left_stick_y/5;
            armPivot.moveToPosition((int)joystickArmPosition);
       } else {
           armPivot.holdPivot();
       }

       if(joystickArmPosition>PIVOTANGLE){
           joystickArmPosition = PIVOTANGLE;
       } else if(joystickArmPosition<0){
           joystickArmPosition = 0;
       }

       if(gamepad2.x){
           armPivot.climb();
       }


        mecanumDrive.drive(-gamepad2.left_stick_y*.75, gamepad2.left_stick_x*.75, -gamepad2.right_stick_x*.75);

    }
}



