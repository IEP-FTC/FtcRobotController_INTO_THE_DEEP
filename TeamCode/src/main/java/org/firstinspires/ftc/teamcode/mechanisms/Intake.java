package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

   private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake 1");
    }

    public void runIntake (boolean forward) {

        if (forward) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(-1);
        }
    }

    public void stopIntake () {
        intakeMotor.setPower(0);
    }
}
