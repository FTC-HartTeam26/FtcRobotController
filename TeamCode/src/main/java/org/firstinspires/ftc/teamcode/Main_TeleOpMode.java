package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver; // Import the driver
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import java.util.Locale;

@TeleOp(name = "Main_TeleOpMode")
public class Main_TeleOpMode extends LinearOpMode {

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

  //sensors
  private GoBildaPinpointDriver odometryComputer;
  double oldTime = 0; //used to calculate loop frequency

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

    //sensor setup
    odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class,"odometry");

    //set up drive motor directions
    back_right.setDirection(DcMotor.Direction.REVERSE);
    front_right.setDirection(DcMotor.Direction.REVERSE);

    // Configure Odometry Pods (Adjust FWD/REV based on your bot)
    odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Define offsets (distance from center of rotation)
    odometryComputer.setOffsets(10.0, -15.0, DistanceUnit.MM);
    odometryComputer.setYawScalar(1.0); // Tune if turning is inaccurate
    odometryComputer.resetPosAndIMU(); // Reset position to (0,0) and heading to 0

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        //Drive
        front_right.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
        back_right.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
        front_left.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);
        back_left.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);

        //Shooting Motors
        if (gamepad1.a) {
          shooter_right.setPower(1);
          shooter_left.setPower(-1);
        } else {
          shooter_right.setPower(0);
          shooter_left.setPower(0);
        }

        //Intake Motors
        if (gamepad1.b) {
          intake.setPower(1);
          hopper.setPower(-.15);
        } else {
          intake.setPower(0);
          hopper.setPower(0);
        }

        //get odometry data
        odometryComputer.update(); // Crucial: Refresh sensor data
        Pose2D pos = odometryComputer.getPosition();
        double X = pos.getX(DistanceUnit.MM);
        double Y = pos.getY(DistanceUnit.MM);
        double H = pos.getHeading(AngleUnit.DEGREES);
        //double Vel_X = odometryComputer.getVelX(DistanceUnit.MM); //uncomment to use velocity
        //double Vel_Y = odometryComputer.getVelY(DistanceUnit.MM);
        //double Vel_H = odometryComputer.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        //Calculate loop frequency, large number = fast = good
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        //telemetry outputs
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.addData("Pinpoint Status", odometryComputer.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odometryComputer.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        telemetry.addData("X Position", X); //positive = forward
        telemetry.addData("Y Position", Y); //positive = strafe left
        telemetry.addData("Heading Deg", H); //positive = counterclockwise
        telemetry.update();
      }
    }
  }
}
