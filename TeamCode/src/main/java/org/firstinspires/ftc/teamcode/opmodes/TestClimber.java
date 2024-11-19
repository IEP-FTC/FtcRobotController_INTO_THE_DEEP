package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestClimber extends OpMode {

    private DcMotor climberL;
    private DcMotor climberR;
    private boolean toggleState = false;
    private boolean aPressed = false;


    @Override
    public void init() {
        climberL = hardwareMap.dcMotor.get("climberL");
        climberR = hardwareMap.dcMotor.get("climberR");

        climberL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberR.setTargetPosition(0);
        climberL.setTargetPosition(0);

        climberL.setPower(0.75);
        climberR.setPower(0.75);
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

            climberR.setTargetPosition(-72*125/30);
            climberL.setTargetPosition(-72*125/30);

        } else {
            climberR.setTargetPosition(0);
            climberL.setTargetPosition(0);
        }

    }
}