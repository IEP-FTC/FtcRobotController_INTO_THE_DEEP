package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBot;

@TeleOp(name="Drive and Claw", group="Linear Opmode")
public class DriveAndClaw extends OpMode {
    ProgrammingBot bot = new ProgrammingBot();

    public int clawState = 0;
    long servoStartTime = 0;
    boolean isServoMoving = false;

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.setServoPosition(1);
    }

    @Override
    public void loop() {
        bot.driveBot(gamepad1, telemetry);

// Declare variables to handle non-blocking timing


        if (gamepad1.a && !isServoMoving) {
            if (clawState == 0) {
                bot.setServoPosition(1);
                clawState = 1;
            } else if (clawState == 1) {
                bot.setServoPosition(0);
                clawState = 0;
            }

            // Start the timer to track servo movement
            servoStartTime = System.currentTimeMillis();
            isServoMoving = true;  // Mark servo as moving
        }

// Check if the servo has had enough time to complete its action
        if (isServoMoving && System.currentTimeMillis() - servoStartTime >= 500) {
            isServoMoving = false;  // Servo is done moving
        }

// Continue allowing driving and other actions while waiting for the servo


    }
}
