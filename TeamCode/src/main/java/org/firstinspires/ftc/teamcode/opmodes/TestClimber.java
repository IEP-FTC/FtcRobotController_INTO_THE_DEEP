package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class TestClimber extends OpMode {

    private DcMotor climberL;
    private DcMotor climberR;
    private boolean toggleState = false;
    private boolean aPressed = false;
    public static double ARM_POWER = 0.50;
    public static int TARGET_ROTATION_TICKS = 72;


    @Override
    public void init() {
        climberL = hardwareMap.dcMotor.get("climberL");
        climberR = hardwareMap.dcMotor.get("climberR");

        climberL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberR.setTargetPosition(0);
        climberL.setTargetPosition(0);

        climberL.setPower(ARM_POWER);
        climberR.setPower(ARM_POWER);
        climberL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {

        if (gamepad1.a && !aPressed) {
            toggleState = !toggleState; // Toggle state
            aPressed = true;           // Set flag to prevent retrigger
        } else if (!gamepad1.a) {
            aPressed = false;          // Reset flag when button is released
        }


        if (toggleState) {

            climberR.setTargetPosition(-TARGET_ROTATION_TICKS*125/30);
            climberL.setTargetPosition(-TARGET_ROTATION_TICKS*125/30);

        } else {
            climberR.setTargetPosition(0);
            climberL.setTargetPosition(0);
        }

    }
}