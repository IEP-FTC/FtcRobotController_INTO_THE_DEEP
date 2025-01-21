package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestServo extends TestItem{
    private Servo servo;

    public TestServo(String description, Servo servo) {
        super(description);
        this.servo = servo;
    }


    @Override
    public void run(boolean on, Telemetry telemetry) {
        if (on) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }

    }

}
