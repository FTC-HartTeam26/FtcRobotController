package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Testing Hardware v1", group="TeleOpModes")
public class Main_TeleOpMode_Testing extends LinearOpMode {
    //Declare class-level objects and variables
    DcMotor hopper_encoder;
    CRServo hopper;
    Servo flipper;
    DcMotor shooter_left;
    DcMotor shooter_right;
    DcMotor intake;
    DcMotor back_left;
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_right;
    GoBildaPinpointDriver odometryComputer;
    double oldTime = 0; //used to calculate loop frequency

    //This is the main method. It runs when you press INIT on the Driver Hub
    //This method also contains the main loop
    @Override
    public void runOpMode() {
        // configure robot
        initializeHardware();
        configureDriveMotors();
        configureOdometry();
        configureHopperEncoder();

        // variables that need to persist for multiple loop cycles
        boolean down_already_pressed = false; //used to store down button status

        // Wait for the game to start (driver presses START)
        telemetry.addData("Init:", "Press Start");
        telemetry.update();
        waitForStart();

        // Main loop
        // this is what runs after you press Start, doesn't stop looping until you press Stop
        while (opModeIsActive()) {

            //this method returns whether the ball is in shooting position
            boolean ball_in_position = false; //reset ball in position variable
            ball_in_position =  readyToLiftBall();

            //this method sets the power of drive motors
            driveRobot();

            //this method resets odometry readings when down arrow is pressed
            down_already_pressed =  resetOdometry(down_already_pressed);

            //this method sets shooter motors power and returns hopper power
            double hopperPower = 0;
            hopperPower = shootBall(ball_in_position, hopperPower);

            //this method sets intake motors and returns hopper power
            hopperPower =  controlIntake(hopperPower);

            //this method raises flipper if up arrow is pressed
            controlFlipper();

            //this method updates odometry readings
            updateOdometry();

            //this method calculates the loop frequency
            updateLoopFrequency();

            //set hopper motor power if required by shooter or intake
            hopper.setPower(hopperPower);
            telemetry.update();
        }
    }

    //Methods for configuring the robot
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
    }

    private void configureDriveMotors() {
        //set up drive motor directions
        back_right.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
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
    }

    private void configureHopperEncoder() {
        // Initialize the hopper encoder
        hopper_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset the encoder to zero
        hopper_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use RUN_USING_ENCODER for velocity control or RUN_TO_POSITION for specific targets
    }

    //Method for controlling the robot during the main loop
    private boolean readyToLiftBall() {
        //define hardware constants
        final double TICKS_PER_REV = 8192.0;
        final double ERROR_MARGIN = TICKS_PER_REV / 100; // Set to how accurate you want the Error Margin to be when spinning hopper.
        double pos1 = TICKS_PER_REV / 3 * 0;
        double pos2 = TICKS_PER_REV / 3 * 1;
        double pos3 = TICKS_PER_REV / 3 * 2;

        //get current hopper position (relative to a full rotation)
        double currPos = hopper_encoder.getCurrentPosition() % TICKS_PER_REV;

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

    private void driveRobot() {
        //FIXME: need to normalize drive motors so they can't exceed 1.0
        front_right.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
        back_right.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
        front_left.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);
        back_left.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);
    }

    private boolean resetOdometry(boolean lastDownArrow) {
        //resetting odometry positions and heading
        boolean currDownArrow = gamepad1.dpad_down;
        if (currDownArrow && !lastDownArrow) {
            odometryComputer.resetPosAndIMU();
        }
        lastDownArrow = currDownArrow;
        return lastDownArrow;
    }

    private double shootBall(boolean ball_is_ready, double  hp) {
        if (gamepad1.a) {
            shooter_right.setPower(1); //turn on shooter motors
            shooter_left.setPower(-1);
            if (!ball_is_ready) {
                hp = -0.15;
            }
        } else {
            shooter_right.setPower(0); //turn off shooter motors
            shooter_left.setPower(0);
        }
        return hp;
    }

    private double controlIntake(double hp) {
        if (gamepad1.b) {
            intake.setPower(1); //turn on intake motor
            hp = -0.15; //set hopper motor power (negative spins clockwise)
        } else {
            intake.setPower(0); //turn off intake motor
        }
        return hp;
    }

    private void controlFlipper() {
        // Lift ball into Shooter using Flipper
        if (gamepad1.dpad_up) {
            flipper.setPosition(0.0);
        } else {
            flipper.setPosition(1.0);
        }
        double flipperPos = flipper.getPosition(); //check flipper position
        telemetry.addData("Flipper Position:", flipperPos);
    }

    private void updateOdometry() {
        odometryComputer.update(); // Crucial: Refresh sensor data
        Pose2D pos = odometryComputer.getPosition();
        double X = pos.getX(DistanceUnit.MM);
        double Y = pos.getY(DistanceUnit.MM);
        double H = pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("X Position",X); //positive = forward
        telemetry.addData("Y Position",Y); //positive = strafe left
        telemetry.addData("Heading Deg",H); //positive = counterclockwise
    }
    private void updateLoopFrequency () {
        //Calculate loop frequency, large number = fast = good
        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = (loopTime == 0) ? 0 : 1 / loopTime;
        oldTime = newTime;
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
    }
}

