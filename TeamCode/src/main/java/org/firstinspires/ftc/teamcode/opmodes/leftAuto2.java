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
public class leftAuto2 extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    ServoIntake intake = new ServoIntake();
    Slide slide = new Slide();

    public static boolean phase1 = true;
    public static boolean phase2 = false;
    public static boolean phase3 = false;
    public static boolean phase4 = false;
    public static boolean phase5 = false;
    public static boolean phase6 = false;
    public static boolean phase7 = false;
    public static boolean phase8 = false;
    public static boolean phase9 = false;
    public static boolean phase10 = false;
    public static boolean phase11 = false;
    public static int C_DRIVE_FORWARD_TICKS =1550;
    public static int E_DRIVE_BACKWARD_TICKS =1300;
    public static int B_ANGLE = 117;
    public static int ANGLE2 = 134;
    public static int GMOVESIDE = 1050;
    public static int IROTATE = 150;
    public static int MSTRAFE = 1200;
    public static int NDRIVE = 800;
    public static int JARM = 70;
    public static int HDRIVE = 1500;
    public static double MROTATE = 0;
    public static double HTIME = .25;
    public double drivePosition;
    private enum Steps{
        aResetArm,
        bArmUp,
        cDriveForward,
        dArmUp,
        e,
        f,
        g,
        h,
        i,
        j,
        k,
        l,
        m,
        n,
        o,
        p,
        q,
        r,
        s,
        t,
        u,
        v,
        w,
        x,
        y,
        z
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
                armPivot.resetArmPosition();
                elapsedTime = timer.seconds();
                if (elapsedTime > .5) {
                    step = Steps.bArmUp;
                    armPivot.resetEncoder();
                }
                break;
            case bArmUp:
                armPivot.moveToAngle(B_ANGLE);
                if ((int) armPivot.getCurrentAngle() >= armPivot.getTargetAngle() - 2) {
                    step = Steps.cDriveForward;
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case cDriveForward:
                if (mecanumDrive.getDrivePosition() < drivePosition + C_DRIVE_FORWARD_TICKS + 50) {
                    mecanumDrive.drive(.8, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.dArmUp;
                    timer.reset();
                }
                break;

            case dArmUp:
                elapsedTime = timer.seconds();
                armPivot.moveToAngle(ANGLE2);
                if ((int) armPivot.getCurrentAngle() >= (int) armPivot.getTargetAngle() - 2 && elapsedTime > .5) {
                    step = Steps.e;
                    drivePosition = mecanumDrive.getDrivePosition();
                    timer.reset();
                }
                telemetry.addData("Timer: ", timer.seconds());
                break;

            case e:
                elapsedTime = timer.seconds();
                if (elapsedTime > 0.5) {
                    armPivot.moveToAngle(armPivot.armRestAngle);
                }
                if (mecanumDrive.getDrivePosition() > drivePosition - E_DRIVE_BACKWARD_TICKS) {
                    mecanumDrive.drive(-.5, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.f;
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
                break;


            case g:
                if (phase1 == true) {
                    if (mecanumDrive.getDrivePosition() <= -GMOVESIDE) {
                        mecanumDrive.stop();
                        step = Steps.h;
                        drivePosition = mecanumDrive.getDrivePosition();
                        timer.reset();
                    } else {
                        drivePosition = mecanumDrive.getDrivePosition();
                        mecanumDrive.drive(0, -.8, .05);
                    }
                }
                break;
            case h:
                elapsedTime = timer.seconds();
                if (phase2 == true && elapsedTime > HTIME) {
                    if (mecanumDrive.getDrivePosition() < drivePosition + HDRIVE) {
                        mecanumDrive.drive(.8, 0, 0);
                    } else {
                        mecanumDrive.stop();
                        step = Steps.i;
                        drivePosition = mecanumDrive.getDrivePosition();
                    }
                }
                break;
            case i:
                if (phase3 == true) {
                    //380 ticks is = 90 degrees for rotation
                    if (mecanumDrive.getDrivePosition() >= -IROTATE) {
                        mecanumDrive.drive(0, 0, -.3);
                    } else {
                        mecanumDrive.stop();
                        step = Steps.j;
                        drivePosition = mecanumDrive.getDrivePosition();
                    }
                }
                break;
            case j:
                if (phase4 == true) {
                    if (armPivot.getCurrentAngle() < JARM) {
                        armPivot.moveToAngle(JARM + 1);
                    } else {
                        step = Steps.k;
                        timer.reset();
                    }
                }
                break;
            case k:
                if (phase5 == true) {
                    elapsedTime = timer.seconds();

                    if (elapsedTime < 2) {
                        slide.autoSlide(true, 0.4);
                        intake.runIntake(true);
                        mecanumDrive.drive(.1, 0, 0);
                    } else {
                        step = Steps.l;
                        intake.stopIntake();
                        slide.stopSlide();
                        timer.reset();
                    }
                }

                telemetry.addData("Timer Seconds", timer.seconds());
                telemetry.update();
                break;

            case l:
                if (phase6 == true) {
                    if (armPivot.getCurrentAngle() < 165) {
                        armPivot.moveToAngle(165);
                    } else {
                        step = Steps.m;
                    }
                }

            case m:
                if (phase7 == true) {
                    if (mecanumDrive.getDrivePosition() > drivePosition - MSTRAFE) {
                        mecanumDrive.drive(0, -.8, MROTATE);
                        armPivot.moveToAngle(165);
                    } else {
                        mecanumDrive.stop();
                        step = Steps.n;
                        drivePosition = mecanumDrive.getDrivePosition();
                        armPivot.holdPivot();
                        timer.reset();
                    }
                }
                break;

            case n:
                if (phase8 == true) {
                    elapsedTime = timer.seconds();
                    if (slide.getSlidePosition() <= 1750) {
                        slide.runSlide(true, 1);
                        armPivot.moveToAngle(165);
                        telemetry.addData("slide position", slide.getSlidePosition());
                        telemetry.update();
                        timer.reset();
                    } else {
                        slide.stopSlide();
                        armPivot.moveToAngle(165);
                        timer.reset();
                        telemetry.addData("slide position", slide.getSlidePosition());
                        telemetry.update();
                        step = Steps.o;
                    }
                }
                break;

            case o:
                if (phase9 == true) {
                    if (mecanumDrive.getDrivePosition() < drivePosition + NDRIVE) {
                        mecanumDrive.drive(.3, 0, 0);
                    } else {
                        mecanumDrive.stop();
                        step = Steps.p;
                        timer.reset();
                    }
                }
                break;

            case p:
                if (phase10 == true) {
                    elapsedTime = timer.seconds();
                    if (elapsedTime < 1.5) {
                        intake.runIntake(false);
                        mecanumDrive.stop();
                    } else {
                        intake.stopIntake();
                        step = Steps.q;
                    }
                }
                break;

            case q:

                if (phase11 == true) {
                    if (slide.getSlidePosition() >= 70) {
                        slide.runSlide(false, 1);
                        mecanumDrive.drive(-.3, 0, 0);
                    } else {
                        slide.stopSlide();
                        armPivot.moveToAngle(65);
                        mecanumDrive.stop();
                    }
                }

                armPivot.addTelemetry(telemetry);
                slide.addTelemetry(telemetry);
                telemetry.addData("slide position", slide.getSlidePosition());
                telemetry.update();

        }
    }
}
