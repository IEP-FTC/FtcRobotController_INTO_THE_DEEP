package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotor slideMotor;

    public void init(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "Slide 1");
    }

    public void stopSlide () {
        slideMotor.setPower(0);
    }
    public void runSlide (boolean extend, double power) { //TODO limit slide extension&retraction length with motor ticks (run using encoder)
        int position = slideMotor.getCurrentPosition();
        if (extend) {
            if (position <= 12.5 * 288) {
                slideMotor.setPower(power);
            } else {
                stopSlide();
            }
        } else {
            if (position >= 0) {
                slideMotor.setPower(-power);
            } else {
                stopSlide();
            }
        }
    }

    public void moveToPosition(int inches) {
        int  targetPosition = inches*(288/3); //Figure out equation for inches/ticks
        slideMotor.setTargetPosition(targetPosition);

    }

    //TODO new function to fully extend/retract slide at full power (from any point)
}
