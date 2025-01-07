package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DriveMotorId extends OpMode {
    DcMotor motor;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop(){
        motor.setPower(.2);
    }

}
