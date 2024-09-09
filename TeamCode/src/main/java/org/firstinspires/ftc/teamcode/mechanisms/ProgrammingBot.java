package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ProgrammingBot {
    public static double SPEED_ADJUSTMENT = 0.5;
    public static double TURN_ADJUSTMENT = 0.5;
    private DigitalChannel touchSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;


    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        leftMotor = hwMap.get(DcMotor.class, "Drive 1");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor = hwMap.get(DcMotor.class, "Drive 2");
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public boolean getTouchSensorState(){
        return touchSensor.getState();
    }

    public void runArcadeDrive(double forwardSpeed, double turnSpeed){
        leftMotor.setPower(forwardSpeed*SPEED_ADJUSTMENT-turnSpeed*TURN_ADJUSTMENT);
        rightMotor.setPower(forwardSpeed*SPEED_ADJUSTMENT+turnSpeed*TURN_ADJUSTMENT);
    }
    public void runTankDrive(double leftSpeed, double rightSpeed){
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }

}
