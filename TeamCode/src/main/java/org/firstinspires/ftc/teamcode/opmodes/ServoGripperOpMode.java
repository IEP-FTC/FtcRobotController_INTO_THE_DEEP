package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.ServoGripper;

public class ServoGripperOpMode extends OpMode {
    ServoGripper gripper = new ServoGripper();

    public void init() {
        gripper.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            gripper.open();
        }

        if (gamepad2.b) {
            gripper.close();
        }
    }
}
