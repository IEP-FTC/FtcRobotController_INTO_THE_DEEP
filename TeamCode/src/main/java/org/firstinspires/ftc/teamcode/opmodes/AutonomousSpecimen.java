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
        DriveBackward,
        Extend,
        Hook,
        NetZone
    }
    private Steps step = Steps.DriveBackward;

    public void init(){
        mecanumDrive.init(hardwareMap);
        armPivot.init(hardwareMap);
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

    }

    public void loop(){
        switch(step){
            case SetAngle:
                armPivot.moveToAngle(116);
                if((int)armPivot.getCurrentAngle() == (int)armPivot.getTargetAngle()){
                    step = Steps.DriveBackward;
                    break;
                }
            case DriveBackward:


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
                //step = Steps.NetZone;
                break;
            case NetZone:
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
