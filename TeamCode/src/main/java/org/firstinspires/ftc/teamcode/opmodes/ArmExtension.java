package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class ArmExtension extends OpMode {

    private DcMotor extensionMotor;

    @Override
    public void init(){
        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");

    }

    @Override
    public void loop(){
        if(gamepad1.y) {
            extensionMotor.setPower(0.5);
        } else if (gamepad1.x){
            extensionMotor.setPower(-0.5);
        } else {
            extensionMotor.setPower(0);
        }
        // use the trigger buttons later to make the speed be different depending on how hard you're pressing the trigger

    }

}
