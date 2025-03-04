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
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@TeleOp
public class MasterMode extends OpMode {
    public static int MAXPIVOTANGLE = 165;
    public static double TARGETANGLE;
    ServoIntake intake = new ServoIntake();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    Slide slide = new Slide();
    public boolean climbMode = false;
    MecanumDrive mecanumDrive = new MecanumDrive();
    boolean wasPressed = false;
    boolean beingPressed = false;


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
        if(!climbMode) {
            if (gamepad1.left_trigger > .1) {
                slide.runSlide(false, gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > .1) {
                slide.runSlide(true, gamepad1.right_trigger);
            } else {
                slide.stopSlide();
            }//TODO add full extension/retract on bumpers
        }

        if(gamepad2.x){
            climbMode=true;
        }
        if(gamepad2.y){
            climbMode=false;
        }

        if (climbMode) {
            slide.climb();
        }

        slide.addTelemetry(telemetry);

        if(!(TARGETANGLE== armPivot.getCurrentAngle())){
            armPivot.moveToAngle(TARGETANGLE);
        }

        if(gamepad1.a){
            TARGETANGLE=MAXPIVOTANGLE;
        }
        if(gamepad1.y && slide.getSlidePosition()<500){
            TARGETANGLE=armPivot.armRestAngle+5;
        }
        if(abs(gamepad1.left_stick_y)>.01){
            TARGETANGLE+=-gamepad1.left_stick_y/2;
        }
        if(TARGETANGLE>MAXPIVOTANGLE){
            TARGETANGLE=MAXPIVOTANGLE;
        }
        if(TARGETANGLE<armPivot.armRestAngle){
            TARGETANGLE=armPivot.armRestAngle;
        }

        beingPressed = gamepad2.a && gamepad2.dpad_down;

        if (beingPressed){
            armPivot.resetArmPosition();
            wasPressed = true;
        }

        if (wasPressed && !beingPressed){
            armPivot.resetEncoder();
            wasPressed = false;
        }
        telemetry.addData("beingPressed", beingPressed);

        telemetry.addData("wasPressed", wasPressed);

        armPivot.addTelemetry(telemetry);

        if(armPivot.getCurrentAngle()>90){
            mecanumDrive.drive(-gamepad2.left_stick_y*.3, gamepad2.left_stick_x*.3, gamepad2.right_stick_x*.3);
        }else {
            mecanumDrive.drive(-gamepad2.left_stick_y * .7, gamepad2.left_stick_x * .5, gamepad2.right_stick_x * .5);
        }

        mecanumDrive.addTelemetry(telemetry);

        telemetry.update();
    }
}



