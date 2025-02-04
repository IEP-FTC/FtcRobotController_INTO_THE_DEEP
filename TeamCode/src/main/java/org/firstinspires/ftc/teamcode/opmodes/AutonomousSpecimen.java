package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
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

    public static int DRIVE_FORWARD_TICKS=1000;
    public double IMU_start;

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
                armPivot.moveToAngle(117);
                if((int)armPivot.getCurrentAngle() >= 115){
                    step = Steps.DriveForward;
                    mecanumDrive.resetDrivePosition();
                }
                break;
            case DriveForward:
                if(mecanumDrive.getDrivePosition()<DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(1, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Extend;
                }
                break;

            case Extend:
                armPivot.moveToAngle(127);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()){
                    step = Steps.Hook;
                    mecanumDrive.resetDrivePosition();
                }
                break;

            case Hook:
                if(mecanumDrive.getDrivePosition()<DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(-1, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Lower_Arm;
                    IMU_start = mecanumDrive.getIMUHeading();
                }
                break;
            case Lower_Arm:
                armPivot.moveToAngle(armPivot.armRestAngle);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()) {
                    step = Steps.Rotate;
                    mecanumDrive.resetDrivePosition();
                }
                break;
            case Rotate:
                if(mecanumDrive.getIMUHeading()<IMU_start+90) {
                    mecanumDrive.drive(0, 0, .5);
                } else {
                    mecanumDrive.stop();
                    step = Steps.ObservationZone;
                    mecanumDrive.resetDrivePosition();
                }
                break;

            case ObservationZone:
                if(mecanumDrive.getDrivePosition()<DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(1, 0, 0);
                } else {
                    mecanumDrive.stop();
                }
                break;

        }

    }
}
