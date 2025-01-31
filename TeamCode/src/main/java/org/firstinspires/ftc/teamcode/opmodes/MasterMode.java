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

    MecanumDrive mecanumDrive = new MecanumDrive();


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

        if (gamepad2.x) {
            slide.climb(false, gamepad2.x);
        }

        if(!(TARGETANGLE== armPivot.getCurrentAngle())){
            armPivot.moveToAngle(TARGETANGLE);
        }

        if(gamepad1.a){
            TARGETANGLE=MAXPIVOTANGLE;
        }
        if(gamepad1.y){
            TARGETANGLE=armPivot.armRestAngle+5;
        }
        if(abs(gamepad1.left_stick_y)>.05){
            TARGETANGLE+=-gamepad1.left_stick_y/2;
        }
        if(TARGETANGLE>MAXPIVOTANGLE){
            TARGETANGLE=MAXPIVOTANGLE;
        }
        if(TARGETANGLE<armPivot.armRestAngle){
            TARGETANGLE=armPivot.armRestAngle;
        }

        armPivot.addTelemetry(telemetry);

        mecanumDrive.drive(-gamepad2.left_stick_y*.75, gamepad2.left_stick_x*.75, -gamepad2.right_stick_x*.75);
        telemetry.update();
    }
}



