package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotor slideMotor;
    private int slidePosition;

    public void init(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "Slide 1");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void stopSlide () {
        slideMotor.setPower(0);
    }
    public void runSlide (boolean extend, double power) { //TODO limit slide extension&retraction length with motor ticks (run using encoder)
        int position = slideMotor.getCurrentPosition();
        slidePosition = position;
        if (extend) {
            if (position <= 1800-10) {
                slideMotor.setPower(power);
            } else {
                stopSlide();
            }
        } else {
            if (position >= 20) {
                slideMotor.setPower(-power);
            } else {
                stopSlide();
            }
        }
    }
    public void climb () { //TODO limit slide extension&retraction length with motor ticks (run using encoder)
        int position = slideMotor.getCurrentPosition();
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidePosition = position;

        runSlide(false, 1);


    }
    public void climb (int ticks) { //TODO limit slide extension&retraction length with motor ticks (run using encoder)
        int position = slideMotor.getCurrentPosition();
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidePosition = position;

        if (slideMotor.getCurrentPosition()<ticks){
            slideMotor.setPower(-1);
        }


    }

    public void moveToPosition(int inches) {
        int  targetPosition = inches*(288/3); //Figure out equation for inches/ticks
        slideMotor.setTargetPosition(targetPosition);

    }
    public int getSlidePosition(){
        return slidePosition;
    }

    //TODO new function to fully extend/retract slide at full power (from any point)
}
