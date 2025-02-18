package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.PIDFArmPivot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;
import org.firstinspires.ftc.teamcode.mechanisms.Slide;

@Config
@Autonomous(preselectTeleOp = "MasterMode")
public class leftAuto extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    PIDFArmPivot armPivot = new PIDFArmPivot();
    ServoIntake intake = new ServoIntake();
    Slide slide = new Slide();
    private DigitalChannel touchSensor;

    public static int C_DRIVE_FORWARD_TICKS =1450;
    public static int B_ANGLE =119;
    public static int ANGLE2=134;
    public static int GMOVESIDE;
    public double IMU_start;
    public double drivePosition;
    public int goIntake;
    public int goSlide;
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
                if (mecanumDrive.getDrivePosition() < drivePosition + C_DRIVE_FORWARD_TICKS) {
                    mecanumDrive.drive(.5, 0, 0);
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
                    armPivot.moveToAngle(B_ANGLE);
                }
                if (mecanumDrive.getDrivePosition() > drivePosition - C_DRIVE_FORWARD_TICKS) {
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
                    // Only reset the timer once transitioning to the next step (g)
                    timer.reset();  // Reset the timer here, as we're starting aResetArm new phase
                }
                break;


            case g:
                if (mecanumDrive.getDrivePosition() <= - GMOVESIDE) {
                    mecanumDrive.stop();
                    step = Steps.h;
                    drivePosition = mecanumDrive.getDrivePosition();
                } else {
                    drivePosition = mecanumDrive.getDrivePosition();
                    mecanumDrive.drive(0, -0.5, 0);
                }
                break;
            case h:
                if (mecanumDrive.getDrivePosition() < drivePosition + 1650) {
                    mecanumDrive.drive(.5, 0, 0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.i;
                    IMU_start = mecanumDrive.getIMUHeading();
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case i:
                //380 ticks is = 90 degrees for rotation
                if (mecanumDrive.getDrivePosition() >= -380) {
                    mecanumDrive.drive(0, 0, -.3);
                } else {
                    mecanumDrive.stop();
                    step = Steps.j;
                    drivePosition = mecanumDrive.getDrivePosition();
                }
                break;
            case j:
              if (armPivot.getCurrentAngle() < 73) {
                  armPivot.moveToAngle(73.5);
                } else {
                  step = Steps.k;
                  goIntake = 1;
                  goSlide = 1;
                  timer.reset();
              }
                break;
            case k:

                elapsedTime = timer.seconds();

                if (goIntake == 1 && goSlide == 1 && elapsedTime < 2) {
                    slide.autoSlide(true, 0.4);
                    intake.runIntake(true);
                    mecanumDrive.drive(.1,0,0);
                } else {
                    step = Steps.l;
                    intake.stopIntake();
                    slide.stopSlide();
                    timer.reset();
                }

                telemetry.addData("goIntake", goIntake);
                telemetry.addData("goSlide", goSlide);
                telemetry.addData("Timer Seconds", timer.seconds());
                telemetry.update();
                break;

            case l:
                if (armPivot.getCurrentAngle() < 165){
                    armPivot.moveToAngle(165);
                } else {
                    step = Steps.m;
                }

            case m:
                if (mecanumDrive.getDrivePosition() > drivePosition - 1450) {
                    mecanumDrive.drive(0, -0.5, 0);
                    armPivot.moveToAngle(165);
                } else {
                    mecanumDrive.stop();
                    step = Steps.n;
                    drivePosition = mecanumDrive.getDrivePosition();
                    armPivot.holdPivot();
                    timer.reset();
                }
                break;

            case n:
                elapsedTime = timer.seconds();
                timer.reset();
                goSlide = 1;
                if (slide.getSlidePosition() <= 1750) {
                   slide.runSlide(true,.8);
                    armPivot.moveToAngle(165);
                    telemetry.addData("slide position", slide.getSlidePosition());
                    telemetry.update();
                    timer.reset();
                } else {

                    slide.stopSlide();
                    step = Steps.o;
                    armPivot.moveToAngle(165);
                    timer.reset();
                    telemetry.addData("slide position", slide.getSlidePosition());
                    telemetry.update();
                    goIntake = 0;
                }
                if (mecanumDrive.getDrivePosition() < drivePosition + 600) {
                    mecanumDrive.drive(.2,0,0);
                 } else {
                    mecanumDrive.stop();
                    timer.reset();
                }

                break;

            case o:
                elapsedTime = timer.seconds();
                if (goIntake == 0 && elapsedTime < 2.5) {
                    intake.runIntake(false);
                } else {
                    intake.stopIntake();
                    step = Steps.p;
                    timer.reset();
                }
                break;

            case p:
                if (mecanumDrive.getDrivePosition() > drivePosition - 100) {
                    mecanumDrive.drive(-.2,0,0);
                } else {
                    mecanumDrive.stop();
                }
                if (slide.getSlidePosition() >= 100) {
                    slide.runSlide(false,.8);
                } else {
                    slide.stopSlide();
                    step = Steps.q;
                }
                break;

            case q:
                if(mecanumDrive.getDrivePosition() < drivePosition + 1700) {
                    mecanumDrive.drive(0.05,.5,0);
                } else {
                    mecanumDrive.stop();
                    step = Steps.r;
                }
                break;

            case r:
                if (mecanumDrive.getDrivePosition() < drivePosition + 400) {
                    mecanumDrive.drive(.2,0,0);
                    armPivot.moveToAngle(73);
                } else {
                    step = Steps.s;
                    mecanumDrive.stop();
                    timer.reset();
                }
                break;

            case s:
                 if (armPivot.getCurrentAngle() > 74) {
                armPivot.moveToAngle(73);
                goIntake = 1;
                goSlide = 1;
                timer.reset();
            } else {
                armPivot.moveToAngle(73);
                step = Steps.t;
            }
            break;
            case t:
                elapsedTime = timer.seconds();
                if (goIntake == 1 && goSlide == 1 && elapsedTime < 1.5) {
                    slide.autoSlide(true, 0.4);
                    intake.runIntake(true);
                    mecanumDrive.drive(.15,0,0);
        } else {
                    intake.stopIntake();
                    slide.stopSlide();
                    step = Steps.u;
                    timer.reset();
                }

        armPivot.addTelemetry(telemetry);
        slide.addTelemetry(telemetry);
        telemetry.addData("slide position", slide.getSlidePosition());
        telemetry.update();

        }
    }
}
