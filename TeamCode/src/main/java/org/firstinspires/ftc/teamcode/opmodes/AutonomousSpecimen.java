package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@Autonomous(preselectTeleOp = "MasterMode")
public class AutonomousSpecimen extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    private DigitalChannel touchSensor;

    public static int DRIVE_FORWARD_TICKS=1490;
    public static int ANGLE1=119;
    public static int ANGLE2=134;
    public double IMU_start;
    public double drivePosition;

    private enum Steps{
        SetAngle,
        DriveForward,
        Extend,
        Hook,
        Lower_Arm,
        Rotate,
        ObservationZone
    }
    private Steps step = Steps.SetAngle;
    private ElapsedTime timer;
    private double elapsedTime;

    public void init(){
        mecanumDrive.init(hardwareMap);
        armPivot.init(hardwareMap);
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.timer = new ElapsedTime();

    }
    /*
    steps:
    1. set the angle to 116 degrees and check to see whether that is true
    2. drive forward for X seconds
    3. set the arm angle to 127 and check
    4. hook the specimen on by driving it backwards
    5. (optional) go get another specimen and repeat
    6. drive to observation zone
    */

    public void loop(){
        telemetry.addData("IMU Heading: ", mecanumDrive.getIMUHeading());

        switch(step){
            case SetAngle:
                armPivot.moveToAngle(ANGLE1);
                if((int)armPivot.getCurrentAngle() >= armPivot.getTargetAngle()-2){
                    step = Steps.DriveForward;
                    drivePosition= mecanumDrive.getDrivePosition();
                }
                break;
            case DriveForward:
                if(mecanumDrive.getDrivePosition()<drivePosition+DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(.3, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Extend;
                    timer.reset();
                }
                break;

            case Extend:
                elapsedTime=timer.seconds();
                armPivot.moveToAngle(ANGLE2);
                if((int)armPivot.getCurrentAngle() >= (int)armPivot.getTargetAngle()-2 && elapsedTime>1){
                    step = Steps.Hook;
                    drivePosition= mecanumDrive.getDrivePosition();
                    timer.reset();
                }
                telemetry.addData("Timer: ", timer.seconds());
                break;

            case Hook:
                elapsedTime= timer.seconds();
                if (elapsedTime>0.5) {
                    armPivot.moveToAngle(ANGLE1);
                }
                if(mecanumDrive.getDrivePosition()>drivePosition-DRIVE_FORWARD_TICKS-150) {
                    mecanumDrive.drive(-.3, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Lower_Arm;
                }
                telemetry.addData("Timer: ", timer.seconds());

                break;
            case Lower_Arm:
                armPivot.moveToAngle(armPivot.armRestAngle);
                if((int)armPivot.getCurrentAngle() <= (int)armPivot.getTargetAngle()+2) {
                    step = Steps.ObservationZone;
                    IMU_start = mecanumDrive.getIMUHeading();
                }
                break;
//            case Rotate:
//                telemetry.addData("IMU Heading: ", mecanumDrive.getIMUHeading());
//
//                if(mecanumDrive.getIMUHeading()>IMU_start-87) {
//                    mecanumDrive.drive(0, 0, .5);
//                } else {
//                    mecanumDrive.stop();
//                    step = Steps.ObservationZone;
//                    drivePosition= mecanumDrive.getDrivePosition();
//                }
//                break;

            case ObservationZone:
                if(mecanumDrive.getDrivePosition()<drivePosition+1550) {
                    mecanumDrive.drive(0, 0.3, 0);
                } else {
                    mecanumDrive.stop();
                }
                break;

        }
        armPivot.addTelemetry(telemetry);
        telemetry.update();

    }
}
