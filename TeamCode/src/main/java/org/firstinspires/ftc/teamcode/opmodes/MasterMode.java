package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@TeleOp
public class MasterMode extends OpMode {
    public static int MAXPIVOTANGLE = 160;
    public static double TARGETANGLE;
    Intake intake = new Intake();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    Slide slide = new Slide();
    boolean toggleA = false, aPressed = false;
    boolean toggleY = false, yPressed = false;

    MecanumDrive mecanumDrive = new MecanumDrive();

    double joystickTargetPosition;

    @Override
    public void init() {
        intake.init(hardwareMap);
        armPivot.init(hardwareMap);
        TARGETANGLE=armPivot.armRestAngle;
        slide.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
//        if (gamepad1.a && !aPressed) {
//            toggleState = !toggleState; // Toggle state
//            aPressed = true;           // Set flag to prevent retrigger
//        } else if (!gamepad1.a) {
//            aPressed = false;          // Reset flag when button is released
//        }

//        if (toggleState) {
//            armPivot.moveToAngle(PIVOTANGLE);//TODO adjust angle to correct (<110)
//
//        } else {
//            armPivot.moveToAngle(armPivot.armRestAngle+3);
//        }

//       if(gamepad1.a){
//           armPivot.moveToAngle(PIVOTANGLE);
//       }
//       if(gamepad1.y){
//           armPivot.moveToAngle(armPivot.armRestAngle+3);
//       }

//       if(abs(gamepad1.left_stick_y)>0.1){
//           joystickArmPosition=armPivot.getTargetAngle()-gamepad1.left_stick_y/2;
//           if(joystickArmPosition>PIVOTANGLE){
//               joystickArmPosition = PIVOTANGLE;
//           } else if(joystickArmPosition<armPivot.armRestAngle){
//               joystickArmPosition = armPivot.armRestAngle;
//           }
//           armPivot.moveToAngle(joystickArmPosition);
//       } else {
//           armPivot.holdPivot();
//       }

        //Pivot Arm Control

//        if (gamepad1.a && !aPressed) {
//            toggleA = !toggleA; // Toggle state
//            aPressed = true;           // Set flag to prevent retrigger
//            toggleY=false;
//        } else if (!gamepad1.a) {
//            aPressed = false;          // Reset flag when button is released
//        }
//
//        if (gamepad1.y && !yPressed) {
//            toggleY = !toggleY; // Toggle state
//            yPressed = true;// Set flag to prevent retrigger
//            toggleA=false;
//        } else if (!gamepad1.y) {
//            yPressed = false;          // Reset flag when button is released
//        }
//
//        if (toggleA) {
//            armPivot.moveToAngle(PIVOTANGLE);
//            toggleY=false;
//        }
//
//        if (toggleY) {
//            armPivot.moveToAngle(armPivot.armRestAngle+5);
//            toggleA=false;
//        }
//
//        if(abs(gamepad1.left_stick_y)>0.1){
//            toggleA=false;
//            toggleY=false;
//            joystickTargetPosition=armPivot.getTargetAngle();
//            if(joystickTargetPosition>PIVOTANGLE){
//               joystickTargetPosition = PIVOTANGLE;
//            } else if(joystickTargetPosition<armPivot.armRestAngle){
//               joystickTargetPosition = armPivot.armRestAngle;
//            }
//            armPivot.moveToAngle(joystickTargetPosition);
//       }

        if(!(TARGETANGLE== armPivot.getCurrentAngle())){
            armPivot.moveToAngle(TARGETANGLE);
        }

        if(gamepad1.a){
            TARGETANGLE=MAXPIVOTANGLE;
        }
        if(gamepad1.y){
            TARGETANGLE=armPivot.armRestAngle+5;
        }
        if(gamepad1.left_stick_y>.05){
            TARGETANGLE+=gamepad1.left_stick_y/2;
        }


        armPivot.addTelemetry(telemetry);




        mecanumDrive.drive(-gamepad2.left_stick_y*.75, gamepad2.left_stick_x*.75, -gamepad2.right_stick_x*.75);
        telemetry.update();
    }
}



