package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Main_Autonomous_Mode", group="AutoMode")
public class Main_Autonomous extends LinearOpMode {
    private DcMotor hopper_encoder;
    private CRServo hopper;
    private Servo flipper;
    private DcMotor shooter_left;
    private DcMotor shooter_right;
    private DcMotor intake;
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_right;
    private GoBildaPinpointDriver odometryComputer;
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Limelight3A limelight;
    private NormalizedColorSensor colorSensorTest; //FIXME: DELETE AFTER TESTING
    private DistanceSensor distanceSensorTest; //FIXME: DELETE AFTER TESTING
    private Servo hood;
    double oldTime = 0; //used to calculate loop frequency
    private int ballShotCount;
    @Override

    public void runOpMode() {

        initializeHardware();
        configureOdometry();
        configureDriveMotors();
        configureOdometry();
        ballShotCount = 0;

        waitForStart();

        while (opModeIsActive()) {

            //this method returns whether the ball is in shooting position
            boolean ball_in_position = false; //reset ball in position variable
            ball_in_position =  readyToLiftBall();

            //this method sets shooter motors power and returns hopper power
            double hp_shooter; //hopper power for aligning ball to flipper
            hp_shooter = shootBall(ball_in_position);

            //set hopper motor power if required by shooter or intake
            hopper.setPower(hp_shooter);

            if (ballShotCount == 3) {
                driveRobot(0.3, 0, -0.2);
                sleep(2000);
                driveRobot(0,0,0);
                ballShotCount = 4;
                sleep(500);
            }
            if (ballShotCount ==4) {
                ball_in_position =  readyToLiftBall();
                if (!ball_in_position) {
                    hopper.setPower(-0.12);
                } else {
                    hopper.setPower(0);
                    ballShotCount = 5;
                }
            }
            if (ballShotCount == 5) {
                return;
            }
        }
    }
    private void driveToPosition_Relative(double dist) {
        double margin_of_error = 5.0; // update error margin as needed
        double drivePower = 0.5; // update drive power as needed

        // Record exact starting coordinates
        double startX = odometryComputer.getPosX(DistanceUnit.MM);
        double startY = odometryComputer.getPosY(DistanceUnit.MM);
        double distanceTraveled = 0;

        // Loop until the straight-line distance matches the target
        while (distanceTraveled < (dist - margin_of_error)) {
            driveRobot(drivePower, 0, 0);
            odometryComputer.update();
            double currX = odometryComputer.getPosX(DistanceUnit.MM);
            double currY = odometryComputer.getPosY(DistanceUnit.MM);
            // Distance formula: sqrt((x2-x1)^2 + (y2-y1)^2)
            distanceTraveled = Math.hypot(currX - startX, currY - startY);
        }
        // Stop the motors immediately after the loop
        driveRobot(0, 0, 0);
    }
    private void rotate_Relative(double deg){
        double margin_of_error = 2.0;
        double kP = 0.03; // Proportional gain: adjusts turn speed based on remaining error
        double minPower = 0.15; // Minimum power needed to overcome drivetrain friction

        odometryComputer.update();
        double startDeg = odometryComputer.getHeading(AngleUnit.DEGREES);
        double targetHeading = startDeg + deg;

        // Normalize target heading to stay between -180 and 180 degrees
        if (targetHeading > 180) targetHeading -= 360;
        else if (targetHeading <= -180) targetHeading += 360;

        while (opModeIsActive()) { // Ensures the loop stops if the stop button is pressed
            odometryComputer.update();
            double currHeading = odometryComputer.getHeading(AngleUnit.DEGREES);

            // Calculate the shortest path to the target angle
            double error = targetHeading - currHeading;
            if (error > 180) error -= 360;
            else if (error <= -180) error += 360;

            // Exit loop if the robot is within the margin of error
            if (Math.abs(error) < margin_of_error) {
                break;
            }

            // Calculate power based on error
            double turnPower = error * kP;

            // Cap the maximum power to 0.7 to maintain control
            turnPower = Math.max(-0.7, Math.min(0.7, turnPower));

            // Add minimum power so the robot doesn't stall near the target
            if (turnPower > 0 && turnPower < minPower) turnPower = minPower;
            if (turnPower < 0 && turnPower > -minPower) turnPower = -minPower;

            driveRobot(0, 0, turnPower);
            sleep(1000);
            telemetry.addData("Start", startDeg);
            telemetry.addData("Current", currHeading);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Power", turnPower);
            telemetry.addData("Heading", odometryComputer.getHeading(AngleUnit.DEGREES));
            telemetry.addData("X", odometryComputer.getPosX(DistanceUnit.MM));
            telemetry.addData("Y", odometryComputer.getPosY(DistanceUnit.MM));
            telemetry.update();
        }
        driveRobot(0, 0, 0); // Stop the robot
    }
    private void initializeHardware() {
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter_right = hardwareMap.get(DcMotor.class, "shooter_right");
        shooter_left = hardwareMap.get(DcMotor.class, "shooter_left");
        hopper = hardwareMap.get(CRServo.class, "hopper");
        flipper = hardwareMap.get(Servo.class, "flipper");
        odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        hopper_encoder = hardwareMap.get(DcMotor.class, "hopper_encoder");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ball_color_sensor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color_sensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //FIXME: DELETE THE TEST SENSOR WHEN DONE TESTING
        colorSensorTest = hardwareMap.get(NormalizedColorSensor.class, "test_color_sensor");
        distanceSensorTest = hardwareMap.get(DistanceSensor.class, "test_color_sensor");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0.8);
    }
    private void configureDriveMotors() {
        //set up drive motor directions
        back_right.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
    }
    private void configureOdometry() {
        // Configure Odometry Pods (Adjust FWD/REV based on your bot)
        odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // arg1 is X-encoder, arg2 is Y-encoder
        odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Define offsets (distance from center of rotation)
        odometryComputer.setOffsets(10.0, -15.0, DistanceUnit.MM);
        odometryComputer.setYawScalar(1.0); // Tune if turning is inaccurate
        odometryComputer.resetPosAndIMU(); // Reset position to (0,0) and heading to 0
        sleep(500);
        odometryComputer.update();
    }
    private void driveRobot(double drive, double strafe, double turn) {

        //configuration for mecanum wheels is the sum of all inputs (see mecanum diagram)
        double fr = drive + strafe + turn;
        double br = drive - strafe + turn;
        double fl = drive - strafe - turn;
        double bl = drive + strafe - turn;

        //max is used to normalize motor power to a max of 1.0
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fr), Math.abs(br)),
                Math.max(Math.abs(fl), Math.abs(bl))
        ));

        front_right.setPower(fr / max);
        back_right.setPower(br / max);
        front_left.setPower(fl / max);
        back_left.setPower(bl / max);
    }
    private double shootBall(boolean ball_is_ready) {
        double hopperPower = 0;
        double shootPower_r = 0;
        double shootPower_l = 0;

        boolean ballReady = readyToLiftBall();
        if (ballShotCount < 3) {

            shootPower_r = -0.6;
            shootPower_l = 0.6;
            shooter_right.setPower(shootPower_r);
            shooter_left.setPower(shootPower_l);

            if (!ballReady) {
                hopperPower = -0.08;
            } else {
                hopper.setPower(0);
                sleep(1000);
                flipper.setPosition(0.3);

                sleep(500);
                flipper.setPosition(0.7);
                ballShotCount = ballShotCount + 1;

                sleep(1000);
                hopper.setPower(-0.12);
                sleep(500);
            }
        }
        else {
            shooter_right.setPower(0);
            shooter_left.setPower(0);
        }
        return hopperPower;
    }
    private boolean readyToLiftBall() {
        //define hardware constants
        final double TICKS_PER_REV = 8192.0;
        final double ERROR_MARGIN = TICKS_PER_REV / 80; // Set to how accurate you want the Error Margin to be when spinning hopper.
        double pos1 = TICKS_PER_REV / 3 * 0;
        double pos2 = TICKS_PER_REV / 3 * 1;
        double pos3 = TICKS_PER_REV / 3 * 2;

        //get current hopper position (relative to a full rotation)
        double absPos = hopper_encoder.getCurrentPosition();
        double currPos = ((absPos % TICKS_PER_REV) + TICKS_PER_REV) % TICKS_PER_REV;

        //calculate how many ticks away we are from each position
        double check_pos2 = Math.abs(currPos - pos2); //using absolute value to make checking error margin easier
        double check_pos3 = Math.abs(currPos - pos3);
        //position 1 is unique because currPos goes from max ticks back to zero
        //this means checking for the margin of error is tricky
        //to fix this we reverse the check position halfway through the rotation
        double currPos1;
        if (currPos < TICKS_PER_REV / 2) {
            currPos1 = currPos;
        } else {
            currPos1 = TICKS_PER_REV - currPos;
        }
        double check_pos1 = Math.abs(currPos1 - pos1);

        //check if ball is in any of the shooting positions, set ball_in_pos accordingly
        boolean ballInPos = false;
        if (check_pos1 <= ERROR_MARGIN) {
            ballInPos = true;
        } else if (check_pos2 <= ERROR_MARGIN) {
            ballInPos = true;
        } else if (check_pos3 <= ERROR_MARGIN) {
            ballInPos = true;
        }
        telemetry.addData("Hopper Encoder:", hopper_encoder.getCurrentPosition());
        telemetry.addData("Ball in Pos", ballInPos);
        return ballInPos;
    }
}
