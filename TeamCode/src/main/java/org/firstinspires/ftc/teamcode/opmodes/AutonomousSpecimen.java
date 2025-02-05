package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@Autonomous(preselectTeleOp = "MasterMode")
public class AutonomousSpecimen extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    private DigitalChannel touchSensor;

    public static int DRIVE_FORWARD_TICKS=1500;
    public static int ANGLE1=119;
    public static int ANGLE2=132;
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

    public void init(){
        mecanumDrive.init(hardwareMap);
        armPivot.init(hardwareMap);
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                    mecanumDrive.drive(.5, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Extend;
                }
                break;

            case Extend:
                armPivot.moveToAngle(ANGLE2);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()-2){
                    step = Steps.Hook;
                    drivePosition= mecanumDrive.getDrivePosition();
                }
                break;

            case Hook:
                if(mecanumDrive.getDrivePosition()>drivePosition-DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(-.5, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Lower_Arm;
                }
                break;
            case Lower_Arm:
                armPivot.moveToAngle(armPivot.armRestAngle);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()+2) {
                    step = Steps.Rotate;
                    IMU_start = mecanumDrive.getIMUHeading();
                }
                break;
            case Rotate:
                if(mecanumDrive.getIMUHeading()>IMU_start-90) {
                    mecanumDrive.drive(0, 0, .5);
                } else {
                    mecanumDrive.stop();
                    step = Steps.ObservationZone;
                    drivePosition= mecanumDrive.getDrivePosition();
                }
                break;

            case ObservationZone:
                if(mecanumDrive.getDrivePosition()<drivePosition+DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(.5, 0, 0);
                } else {
                    mecanumDrive.stop();
                }
                break;

        }
        armPivot.addTelemetry(telemetry);
        telemetry.addData("IMU Heading: ", mecanumDrive.getIMUHeading());
        telemetry.update();

    }
}
