package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libraries.PIDFController;
@Config
public class MecanumDrive {
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    ImuControl imu = new ImuControl();
    private PIDFController pid;
    public static double kP=0.0, kI=0.0, kD=0.00;


    private double targetHeading = 0, currentHeading, rotatePower;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");   //expansion hub - 3
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor"); //expansion hub - 2
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");     //expansion hub - 0
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");   //expansion hub - 1

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.init(hardwareMap);
        pid = new PIDFController(0.0, 0, 0);
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
    // Uses PID loop from Gyro to correct for rotation errors (when there is no rotation input)
    public void pidDrive(double forward, double right, double rotate){
        if (rotate <= 0.01 && rotate >= -0.01){
            currentHeading = imu.getHeading(AngleUnit.DEGREES);
            pid.setCoefficients(kP,kI,kD);
            rotatePower = pid.update(targetHeading, currentHeading);
            drive(forward,right,rotatePower);
        }else {
            drive(forward, right, rotate);
            targetHeading = imu.getHeading(AngleUnit.DEGREES);
        }
    }
    public void addTelemetry(Telemetry telemetry){
        telemetry.addLine("Drive PID Info");
        telemetry.addData("currentHeading", currentHeading);
        telemetry.addData("targetHeading",targetHeading);
        telemetry.addData("Rotate Power",rotatePower*100);
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
        pidDrive(forward, 0, rotate);

    }
    public double ticksToDriveRotations(double ticks) {
        return ticks/288;
    }

    public int getDrivePosition(){
       return frontLeftMotor.getCurrentPosition();
    }
    public void resetDrivePosition(){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
