package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoIntake {

   private CRServo intakeServo;

    public void init(HardwareMap hardwareMap) {

        intakeServo = hardwareMap.get(CRServo.class, "servo");

    }

    public void runIntake (boolean forward) {

        if (forward) {
            intakeServo.setPower(1);

        } else {
            intakeServo.setPower(-1);
        }
    }
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Intake Power", intakeServo.getPower());
    }
    public void stopIntake () {
        intakeServo.setPower(0);
    }
}
