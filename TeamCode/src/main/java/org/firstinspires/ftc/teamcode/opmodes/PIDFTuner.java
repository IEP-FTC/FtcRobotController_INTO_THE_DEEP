package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libraries.PIDFController;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@TeleOp
public class PIDFTuner extends OpMode {
    public static int PIVOT_ANGLE = 120;
    Intake intake = new Intake();
    PIDFArmPivot armPivot = new PIDFArmPivot();
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void loop() {

        //Pivot Arm Control

        if (gamepad1.a && !aPressed) {
            toggleState = !toggleState; // Toggle state
            aPressed = true;           // Set flag to prevent retrigger
        } else if (!gamepad1.a) {
            aPressed = false;          // Reset flag when button is released
        }

        if (toggleState) {
            armPivot.moveToAngle(PIVOT_ANGLE);


        } else {
            armPivot.moveToAngle(armPivot.armRestAngle);

        }


        armPivot.addTelemetry(telemetry);

        if(abs(gamepad2.left_stick_y)>abs(gamepad2.left_stick_x)){
            mecanumDrive.pidDrive(-gamepad2.left_stick_y, 0, 0);
        } else if(abs(gamepad2.left_stick_y)<abs(gamepad2.left_stick_x)){
            mecanumDrive.pidDrive(0, gamepad2.left_stick_x, 0);
        } else {
            mecanumDrive.pidDrive(0,0, -gamepad2.right_stick_x * .75);
        }
        //mecanumDrive.addTelemetry(telemetry);


        telemetry.update();
    }
}



