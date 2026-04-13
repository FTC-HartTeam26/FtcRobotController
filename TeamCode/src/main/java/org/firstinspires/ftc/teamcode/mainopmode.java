package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "main_opmode")
public class mainopmode extends LinearOpMode {

  //motors
  private DcMotor back_right;
  private DcMotor front_right;
  private DcMotor front_left;
  private DcMotor back_left;
  private DcMotor intake;
  private DcMotor shooter_right;
  private DcMotor shooter_left;

  //servos
  private CRServo hopper;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {

    //motor set up
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    intake = hardwareMap.get(DcMotor.class, "intake");
    shooter_right = hardwareMap.get(DcMotor.class, "shooter_right");
    shooter_left = hardwareMap.get(DcMotor.class, "shooter_left");

    //servo setup
    hopper = hardwareMap.get(CRServo.class, "hopper");

    //set up drive motor directions
    back_right.setDirection(DcMotor.Direction.REVERSE);
    front_right.setDirection(DcMotor.Direction.REVERSE);

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        //drive
        front_right.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
        back_right.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
        front_left.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);
        back_left.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);

        //makes ball go weeeeeeeeeee!!!!!!
        if (gamepad1.a){
          shooter_right.setPower(1);
        }else{
          shooter_right.setPower(0);
        }

        if (gamepad1.a){
          shooter_left.setPower(-1);
        }else{
          shooter_left.setPower(0);
        }

        //intake motion
        if (gamepad1.b){
          intake.setPower(1);
          hopper.setPower(-.15);
        }else{
          intake.setPower(0);
          hopper.setPower(0);
        }



        //telemetry data
        telemetry.addData("gamepad 1 left stick x", gamepad1.left_stick_y + 1);
        telemetry.update();
        //previousleftPos = lift_left.getCurrentPosition();
      }
    }
  }
}
