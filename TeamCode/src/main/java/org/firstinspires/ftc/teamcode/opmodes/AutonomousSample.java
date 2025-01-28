package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;


@Autonomous(preselectTeleOp = "MasterMode")
public class AutonomousSample extends OpMode{
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    Slide slide = new Slide();
    private DigitalChannel touchSensor;
    private enum Steps{
        DriveBackward,
        RotateAndExtend,
        Drop,
        NetZone

    }
    private Steps step = Steps.DriveBackward;

    public void init(){
        mecanumDrive.init(hardwareMap);
        armPivot.init(hardwareMap);
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        slide.init(hardwareMap);
    }

    public void loop(){
        switch(step){
            case DriveBackward:
                if(!touchSensor.getState()){
                    mecanumDrive.drive(-1,0,0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.RotateAndExtend;
                }
            case RotateAndExtend:
                mecanumDrive.drive(0,0,1); //TEST do we need to change this?
                try {
                    Thread.sleep(500); //TEST edit the timing on this
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                mecanumDrive.stop();
                //extension code here

        }

    }
}
