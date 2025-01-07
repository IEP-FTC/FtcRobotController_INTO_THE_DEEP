package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    ImuControl imu = new ImuControl();

    private double targetHeading = 0;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");   //expansion hub - 3
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor"); //expansion hub - 2
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");     //expansion hub - 0
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");   //expansion hub - 1

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.init(hardwareMap);
    }
    private void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /=maxSpeed;
        frontRightPower /=maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    public void drive(double forward, double right, double rotate){
        forward = -forward;
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public void driveGyroCorrected(double forward, double right, double rotate){
        forward = -forward;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        if (rotate <= 0.05){
            double heading = imu.getHeading(AngleUnit.DEGREES);
            double rotateDegrees = targetHeading - heading;
            rotate = rotateDegrees/45;
            frontLeftPower = forward + right + rotate;
            frontRightPower = forward - right - rotate;
            backLeftPower = forward - right + rotate;
            backRightPower = forward + right - rotate;
        }else {
            frontLeftPower = forward + right + rotate;
            frontRightPower = forward - right - rotate;
            backLeftPower = forward - right + rotate;
            backRightPower = forward + right - rotate;
            targetHeading = imu.getHeading(AngleUnit.DEGREES);
        }
        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public void stop(){
        setPowers(0,0,0,0);
    }

    public void goToBearingAndRange(double bearing, double range){
        double rotate;
        double forward;
        double heading = imu.getHeading(AngleUnit.DEGREES);
        double targetHeading = heading+bearing;
        if (targetHeading-heading>5){
            if (bearing>0){
                rotate=.5;
            } else {
                rotate = -0.5;
            }
        } else {
            rotate = 0;
        }
        if (range >10){
            forward = 1;
        } else {
            forward = 0;
        }
        driveGyroCorrected(forward, 0, rotate);

    }
}
