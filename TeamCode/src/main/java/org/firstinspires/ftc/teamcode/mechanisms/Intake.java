package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

   private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake 1");
    }

    public void runIntake (boolean forward, double power) {

        if (forward) {
            intakeMotor.setPower(power);
        } else {
            intakeMotor.setPower(-power);
        }
    }

    public void stopIntake () {
        intakeMotor.setPower(0);
    }
}
