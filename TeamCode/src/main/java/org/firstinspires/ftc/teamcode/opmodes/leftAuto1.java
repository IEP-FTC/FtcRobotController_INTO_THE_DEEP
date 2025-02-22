package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;

@Config
@Autonomous(preselectTeleOp = "MasterMode")
public class leftAuto1 extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    ServoIntake intake = new ServoIntake();
    Slide slide = new Slide();

    public static boolean phase1 = true;
    public static boolean phase2 =  true;
    public static boolean phase3 = true;
    public static boolean phase4 = true;
    public static boolean phase5 = true;
    public static int C_DRIVE_FORWARD_TICKS =1500;
    public static int E_DRIVE_BACKWARD_TICKS =1350;
    public static int B_ANGLE = 118;
    public static int ANGLE2 = 134;
    public double drivePosition;
    private enum Steps{
        aResetArm,
        bArmUp,
        cDriveForward,
        dArmUp,
        e,
        f,
        g,
    }
    private Steps step = Steps.aResetArm;
    private ElapsedTime timer;
    private double elapsedTime;

    public void init(){
        mecanumDrive.init(hardwareMap);
        intake.init(hardwareMap);
        slide.init(hardwareMap);
        armPivot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.timer = new ElapsedTime();

    }

    public void start(){
        telemetry.addData("IMU Heading: ", mecanumDrive.getIMUHeading());
        telemetry.addData("Step", step);
        timer.reset();
    }

    public void loop(){
        telemetry.addData("IMU Heading: ", mecanumDrive.getIMUHeading());
        telemetry.addData("Step", step);
        telemetry.addData("drivePos", drivePosition);
        switch(step) {
            case aResetArm:
                if(phase1 == true) {
                    armPivot.resetArmPosition();
                    elapsedTime = timer.seconds();
                    if (elapsedTime > .5) {
                        step = Steps.bArmUp;
                        armPivot.resetEncoder();
                    }
                }
                break;

            case bArmUp:
                if(phase2 == true) {
                    armPivot.moveToAngle(B_ANGLE);
                    if ((int) armPivot.getCurrentAngle() >= armPivot.getTargetAngle() - 2) {
                        step = Steps.cDriveForward;
                        drivePosition = mecanumDrive.getDrivePosition();
                    }
                }
                break;

            case cDriveForward:
                if (phase3 == true) {
                    if (mecanumDrive.getDrivePosition() < drivePosition + C_DRIVE_FORWARD_TICKS + 50) {
                        mecanumDrive.drive(.8, 0,0);
                    } else {
                        mecanumDrive.stop();
                        step = Steps.dArmUp;
                        timer.reset();
                    }
                }
                break;

            case dArmUp:
                if(phase4 == true) {
                    elapsedTime = timer.seconds();
                    armPivot.moveToAngle(ANGLE2);
                    if ((int) armPivot.getCurrentAngle() >= (int) armPivot.getTargetAngle() - 2 && elapsedTime > .5) {
                        step = Steps.e;
                        drivePosition = mecanumDrive.getDrivePosition();
                        timer.reset();
                    }
                }
                telemetry.addData("Timer: ", timer.seconds());
                break;

            case e:
                if(phase5 == true) {
                    elapsedTime = timer.seconds();
                    if (elapsedTime > 0.5) {
                        armPivot.moveToAngle(armPivot.armRestAngle);
                    }
                    if (mecanumDrive.getDrivePosition() > drivePosition - E_DRIVE_BACKWARD_TICKS) {
                        mecanumDrive.drive(-.8, 0, 0);
                    } else {
                        mecanumDrive.stop();
                        step = Steps.f;
                    }
                }
                telemetry.addData("Timer: ", timer.seconds());

                break;
//
            case f:
                armPivot.moveToAngle(armPivot.armRestAngle);
                if ((int) armPivot.getCurrentAngle() <= (int) armPivot.getTargetAngle() + 2) {
                    step = Steps.g;
                    drivePosition = mecanumDrive.getDrivePosition();
                    timer.reset();
                }

                armPivot.addTelemetry(telemetry);
                slide.addTelemetry(telemetry);
                telemetry.addData("slide position", slide.getSlidePosition());
                telemetry.update();

        }
    }
}
