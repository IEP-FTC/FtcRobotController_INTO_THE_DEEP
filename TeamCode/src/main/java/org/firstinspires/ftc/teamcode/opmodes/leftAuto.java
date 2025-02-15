package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
@Autonomous(preselectTeleOp = "MasterMode")
public class leftAuto extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    private DigitalChannel touchSensor;

    public static int DRIVE_FORWARD_TICKS=1400;
    public static int ANGLE1=119;
    public static int ANGLE2=134;
    public double IMU_start;
    public double drivePosition;

    private enum Steps{
        a,
        b,
        c,
        d,
        e,
        f,
        g,
        h,
        i,
        j,
        k,
        l
    }
    private Steps step = Steps.a;
    private ElapsedTime timer;
    private double elapsedTime;

    public void init(){
        mecanumDrive.init(hardwareMap);
        armPivot.init(hardwareMap);
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
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

        switch(step) {
            case a:
                armPivot.resetArmPosition();
                elapsedTime = timer.seconds();
                if (elapsedTime > 1) {
                    step = Steps.b;
                    armPivot.resetEncoder();
                }
                break;
            case b:
                armPivot.moveToAngle(ANGLE1);
                if ((int) armPivot.getCurrentAngle() >= armPivot.getTargetAngle() - 2) {
                    step = Steps.c;
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case c:
                if (mecanumDrive.getDrivePosition() < drivePosition + DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(.3, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.d;
                    timer.reset();
                }
                break;

            case d:
                elapsedTime = timer.seconds();
                armPivot.moveToAngle(ANGLE2);
                if ((int) armPivot.getCurrentAngle() >= (int) armPivot.getTargetAngle() - 2 && elapsedTime > 1) {
                    step = Steps.e;
                    drivePosition = mecanumDrive.getDrivePosition();
                    timer.reset();
                }
                telemetry.addData("Timer: ", timer.seconds());
                break;

            case e:
                elapsedTime = timer.seconds();
                if (elapsedTime > 0.5) {
                    armPivot.moveToAngle(ANGLE1);
                }
                if (mecanumDrive.getDrivePosition() > drivePosition - DRIVE_FORWARD_TICKS - 200) {
                    mecanumDrive.drive(-.3, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.f;
                }
                telemetry.addData("Timer: ", timer.seconds());

                break;
//            case f:
//                armPivot.moveToAngle(armPivot.armRestAngle);
//                if ((int) armPivot.getCurrentAngle() <= (int) armPivot.getTargetAngle() + 2) {
//                    step = Steps.g;
//                    drivePosition = mecanumDrive.getDrivePosition();
//                }
//                break;
            case f:
                armPivot.moveToAngle(armPivot.armRestAngle);
                if ((int) armPivot.getCurrentAngle() <= (int) armPivot.getTargetAngle() + 2) {
                    step = Steps.g;
                    drivePosition = mecanumDrive.getDrivePosition();
                    // Only reset the timer once transitioning to the next step (g)
                    timer.reset();  // Reset the timer here, as we're starting a new phase
                }
                break;


            case g:
                if (mecanumDrive.getDrivePosition() < drivePosition - 1200) {
                    mecanumDrive.stop();
                    step = Steps.h;
                    drivePosition = mecanumDrive.getDrivePosition();
                } else {
                    drivePosition = mecanumDrive.getDrivePosition();
                    mecanumDrive.drive(0, -0.3, 0);
                }
                break;
            case h:
                if (mecanumDrive.getDrivePosition() < drivePosition + 1000) {
                    mecanumDrive.drive(.3, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.i;
                    IMU_start = mecanumDrive.getIMUHeading();
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case i:
                if (mecanumDrive.getIMUHeading() <  IMU_start + 87) {
                    mecanumDrive.drive(0, 0, .3);
                } else {
                    mecanumDrive.stop();
                    step = Steps.j;
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case j:
                if (mecanumDrive.getDrivePosition() > drivePosition + 100) {
                    mecanumDrive.drive(.3, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.k;
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case k:
                if (mecanumDrive.getDrivePosition() < drivePosition - 1000) {
                    mecanumDrive.drive(0, -0.3, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.l;
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
        }

        armPivot.addTelemetry(telemetry);
        telemetry.update();

    }
}
