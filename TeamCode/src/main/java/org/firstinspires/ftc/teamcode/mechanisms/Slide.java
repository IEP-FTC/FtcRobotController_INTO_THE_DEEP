package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotor slideMoter;

    public void init(HardwareMap hwMap) {

        slideMoter = hwMap.get(DcMotor.class, "Slide 1");
    }

    public void runSlide (boolean extend, double power) {
        if (extend) {
            slideMoter.setPower(power);
        } else {
            slideMoter.setPower(-power);
        }
    }
}
