package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "officialOpMode 2023")
public class OfficialOpMode extends LinearOpMode {

  private Servo launcher;
  private DcMotor Drive1;
  private DcMotor Drive2;
  private DcMotor Intake;
  private DcMotor Climber;
  
  String arm_statis;
  int Angle_1;
  String Claw1_status;
  int Angle_2;
  String Claw2_status;



  /**
   * Describe this function...
   */
  private void Arcade() {
    Drive1.setDirection(DcMotorSimple.Direction.REVERSE);
    Drive1.setPower(gamepad1.left_stick_y);
    Drive2.setPower(gamepad1.right_stick_y);
  }

  /**
   * Describe this function...
   */
  private void climber() {
    if ((gamepad1.left_bumper && gamepad1.right_bumper) == false) {
      Climber.setPower(0);
    }
    if (gamepad1.left_bumper) {
      Climber.setPower(1);
    }
    if (gamepad1.right_bumper) {
      Climber.setPower(-1);
    }
  }
  
  
  @Override
  public void runOpMode() {
    Drive1 = hardwareMap.get(DcMotor.class, "Drive 1");
    Drive2 = hardwareMap.get(DcMotor.class, "Drive 2");
    Intake = hardwareMap.get(DcMotor.class, "Intake");
    Climber = hardwareMap.get(DcMotor.class, "Climber");
    launcher = hardwareMap.get(Servo.class, "Launcher");

    // Put initialization blocks here.
    launcher.setPosition(1);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        climber();
        launcher();
        Arcade();
        intake();
        telemetry.update();
        
      }
    }
  }

    
  

  
  
  private void launcher(){
    if (gamepad2.dpad_up) {
      launcher.setPosition(0.5);
    }
    
  }
  
  private void intake(){
    if ((gamepad2.a && gamepad2.b) == false){
      Intake.setPower(0);
    }
    
    if (gamepad2.a){
      Intake.setPower(1);
    }
    if (gamepad2.b){
      Intake.setPower(-1);
    }
  }
}