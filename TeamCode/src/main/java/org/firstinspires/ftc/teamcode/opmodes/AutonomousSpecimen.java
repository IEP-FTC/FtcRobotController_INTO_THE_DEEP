package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.ArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(preselectTeleOp = "MasterMode")
public class AutonomousSpecimen extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    ArmPivot armPivot = new ArmPivot();
    private DigitalChannel touchSensor;

    private enum Steps{
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
            case DriveBackward:
                if(!touchSensor.getState()) {
                    mecanumDrive.drive(1, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.Extend;
                }

            case Extend:
                armPivot.moveToPosition(90); //TEST edit the angle on this
                //rotate the arm to an angle so as to put the thing on
                //is extension required? test
                step = Steps.Hook;
            case Hook:
                mecanumDrive.drive(1,0,0);
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.stop();
                step = Steps.NetZone;
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



        }



    }
}
