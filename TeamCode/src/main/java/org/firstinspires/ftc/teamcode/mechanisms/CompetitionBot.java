package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

@Config
public class CompetitionBot {

    MecanumDrive mecanumDrive = new MecanumDrive();
    public void init(HardwareMap hwMap) {
        mecanumDrive.init(hwMap);
    }

    public void drive(double forward, double right, double rotate){
        mecanumDrive.drive(forward, right, rotate);
    }

    public void stop(){
        mecanumDrive.drive(0,0,0);
    }


}