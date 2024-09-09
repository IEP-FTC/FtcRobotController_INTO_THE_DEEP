package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBot {
    private DigitalChannel touchSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DifferentialDrive m_drive;

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
        double speedAdjustment = 0.5;
        leftMotor.setPower(forwardSpeed*speedAdjustment-turnSpeed*speedAdjustment);
        rightMotor.setPower(forwardSpeed*speedAdjustment+turnSpeed*speedAdjustment);
    }
    public void runTankDrive(double leftSpeed, double rightSpeed){
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }
}
