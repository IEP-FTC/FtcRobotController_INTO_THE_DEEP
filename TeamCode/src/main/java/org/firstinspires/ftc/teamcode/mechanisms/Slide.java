package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {
    private DcMotor slideMotor;
    private int slidePosition;

    public void init(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "Slide 1");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if(position>=1000) {
            runSlide(false, 1);
        }


    }

    public void moveToPosition(int inches) {
        int  targetPosition = inches*(288/3); //Figure out equation for inches/ticks
        slideMotor.setTargetPosition(targetPosition);

    }
    public int getSlidePosition(){
        return slidePosition;
    }
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Slide Position", slidePosition);
        telemetry.addData("Slide Power",slideMotor.getPower());
    }

    public void autoSlide (boolean extend, double power) { //TODO limit slide extension&retraction length with motor ticks (run using encoder)
        int position = slideMotor.getCurrentPosition();
        slidePosition = position;
        if (extend) {
            if (position <= 1200) {
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
}

