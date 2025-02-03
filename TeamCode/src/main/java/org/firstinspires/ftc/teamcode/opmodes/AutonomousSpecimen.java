package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(preselectTeleOp = "MasterMode")
public class AutonomousSpecimen extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    private DigitalChannel touchSensor;

    private enum Steps{
        SetAngle,
        DriveForward,
        Extend,
        Hook,
        Repeat,
        ObservationZone
    }
    private Steps step = Steps.SetAngle;

    boolean doneOnce = false;

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
                armPivot.moveToAngle(116);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()){
                    step = Steps.DriveForward;
                    break;
                }
            case DriveForward:
                mecanumDrive.drive(1, 0, 0);
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.stop();
                step = Steps.Extend;
                break;

            case Extend:
                armPivot.moveToAngle(127);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()){
                    step = Steps.Hook;
                    break;
                }

            case Hook:
                mecanumDrive.drive(-1,0,0);
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.stop();
                if(!doneOnce){
                    step = Steps.Repeat;
                } else {
                    step = Steps.ObservationZone;
                }
                break;

            case Repeat:
                mecanumDrive.drive(-1,0,0); //go backwards
                try {
                    Thread.sleep(500); //TEST
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(0,0,1); //rotate
                try {
                    Thread.sleep(500); //TEST
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(0,-1,0); //sideways to observation zone, maybe reuse for the parking code
                try {
                    Thread.sleep(500); //TEST
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(1,0,0); //clip the specimen
                try {
                    Thread.sleep(500); //TEST
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(-1,0,0); //go backwards
                try {
                    Thread.sleep(500); //TEST
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(0,0,1); //rotate
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(0,0,1); //right to reset to original position
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                doneOnce = true;
                step = Steps.SetAngle;
                break;

            case ObservationZone:
                mecanumDrive.drive(1,0,0);
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.drive(1,0,0);
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.stop();
                break;










        }



    }
}
