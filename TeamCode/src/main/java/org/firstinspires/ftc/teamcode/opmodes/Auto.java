package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mechanisms.ArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;
@Disabled
@Autonomous
public class Auto extends OpMode {
    MecanumDrive mDrive = new MecanumDrive();
    ArmPivot armPivot = new ArmPivot();

    Slide slide = new Slide();
    private DigitalChannel touchSensor;

    public enum AutoSteps{
        BACK_TO_SUBMERSIBLE,
        HANG_SPECIMEN,
        YELLOW_SAMPLES,
        FORWARD,
        Park, STOP
    }

    private AutoSteps step = AutoSteps.BACK_TO_SUBMERSIBLE;
    boolean drive1 = true;

    @Override
    public void init(){
        mDrive.init(hardwareMap);
        armPivot.init(hardwareMap);
        slide.init(hardwareMap);
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


    }
    @Override
    public void loop(){
        switch (step) {
            case BACK_TO_SUBMERSIBLE:
                armPivot.moveToPosition(140);
                slide.moveToPosition(6);

                if (!touchSensor.getState()) {
                    mDrive.drive(-1, 0, 0);
                } else {
                    mDrive.stop();
                    step = AutoSteps.HANG_SPECIMEN;
                }

            case HANG_SPECIMEN:
                armPivot.moveToPosition(150);
                slide.moveToPosition(-6);
                step = AutoSteps.YELLOW_SAMPLES;

            case YELLOW_SAMPLES:
                armPivot.moveToPosition(-150);
                step = AutoSteps.Park;

            case Park:
                if (drive1) {
                    mDrive.drive(1,0,0);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    drive1 = false;
                } else {
                    mDrive.drive(0, -1,0);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    mDrive.stop();
                }

                //move backwards to obzone

                //see apriltag? or do we just move?
                //add a rotate function to see an april tag
                // rotate to the right 90 degrees
                //then see the tag and use that to position to move block
        }
        //Step 1: Hang Specimen
            // Start facing wall?
            // Lift up the Arm Pivot+Slide
            // Drive toward submersible
            // Stop driving (How to know when to stop.)
            // Hang specimen (retract slide/pivot)

        //Step 2: Push Yellow Samples to Net Zone
            // Check for April Tags
            // repeat pattern of pushing

        //Step 3: Park
            //V1. Park in Observation Zone
            //V2. Park in Ascent Zone

    }
}
