package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestClimber extends OpMode {

    private DcMotor climberL;
    private DcMotor climberR;

    @Override
    public void init() {
        climberL = hardwareMap.dcMotor.get("climberL");
        climberR = hardwareMap.dcMotor.get("climberR");

    }

    @Override
    public void loop() {

        if (gamepad1.a) {

            climberR.setTargetPosition(-72*120/15);
            climberL.setTargetPosition(-72*120/15);
            climberL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            climberR.setTargetPosition(0);
            climberL.setTargetPosition(0);
            climberL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}