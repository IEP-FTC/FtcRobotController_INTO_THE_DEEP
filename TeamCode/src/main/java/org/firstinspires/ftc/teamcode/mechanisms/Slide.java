package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotor slideMotor;
    public void init(HardwareMap HWmap) {
        slideMotor = HWmap.get(DcMotor.class,"Slide");
    }
    public void runSlide(boolean extend, double power) {
        if (extend) {
            slideMotor.setPower(power);
        } else {
            slideMotor.setPower(-power);

        }
    }
}
