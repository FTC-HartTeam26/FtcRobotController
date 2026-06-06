package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name="Testing_Luke", group="TeleOpModes")
public class Testing_Luke extends LinearOpMode {
    //Declare class-level objects and variables
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
    double oldTime = 0; //used to calculate loop frequency
    private Servo hood;

    //This is the main method. It runs when you press INIT on the Driver Hub
    //This method also contains the main loop
    @Override
    public void runOpMode() {
        // configure robot
        initializeHardware();
        configureDriveMotors();
        configureOdometry();
        //initializeHopper(); //FIXME: UNCOMMENT WHEN DISTANCE SENSOR FIXED
        configureHopperEncoder();
        configureLimeLight(8);

        // variables that need to persist for multiple loop cycles
        boolean down_already_pressed = false; //used to store down button status
        boolean up_already_pressed = false; //used to store up button status


        // Wait for the game to start (driver presses START)
        telemetry.addData("Init:", "Press Start");
        telemetry.update();
        waitForStart();

        // Main loop
        // this is what runs after you press Start, doesn't stop looping until you press Stop
        while (opModeIsActive()) {

            //this method returns the results of the ball color sensor
            String ballColor;
            ballColor = getBallColor(colorSensor);

            //FIXME: delete this section after testing is complete
            //this method returns the value of the test color sensor
            String ballColor_Test;
            ballColor_Test = getBallColor(colorSensorTest);

            //this method returns whether the ball is in shooting position
            boolean ball_in_position = false; //reset ball in position variable
            ball_in_position =  readyToLiftBall();

            //this method sets the power of drive motors
            driveRobot();

            //this method resets odometry readings when down arrow is pressed
            //variable used to make sure the reset only happens once per button press
            down_already_pressed =  hood_down (down_already_pressed);
            up_already_pressed =  hood_up (up_already_pressed);

            //this method sets shooter motors power and returns hopper power
            double hp_shooter; //hopper power for aligning ball to flipper
            hp_shooter = shootBall(ball_in_position);

            //this method sets intake motors and returns hopper power
            double hp_intake; //hopper power for ball intake
            hp_intake =  controlIntake();

            //this method raises flipper if up arrow is pressed
            //controlFlipper();

            //this method updates odometry readings
            updateOdometry();

            //this method calculates the loop frequency
            updateLoopFrequency();

            //this method sends limelight goal tracking results to telemetry
            print_limelight_results();

            //set hopper motor power if required by shooter or intake
            hopper.setPower(Math.min(hp_intake, hp_shooter));

            //FIXME: DELETE THIS CODE AFTER TESTING
            telemetry.addData("Ball Color", ballColor);
            telemetry.addData("Test Sensor", ballColor_Test);
            String ballDist = String.format("%.2f", distanceSensor.getDistance(DistanceUnit.CM));
            String distBaseline = String.format("%.2f", distanceSensorTest.getDistance(DistanceUnit.CM));
            telemetry.addData("Ball Dist", ballDist);
            telemetry.addData("Dist Baseline", distBaseline);
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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "ball_color_sensor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color_sensor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //FIXME: DELETE THE TEST SENSOR WHEN DONE TESTING
        colorSensorTest = hardwareMap.get(NormalizedColorSensor.class, "test_color_sensor");
        distanceSensorTest = hardwareMap.get(DistanceSensor.class, "test_color_sensor");
        hood = hardwareMap.get(Servo.class, "hood");
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
        hopper_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //there is no motor
    }

    private void configureLimeLight(int pipeLine) {
        limelight.pipelineSwitch(pipeLine);
        limelight.start();
    }

    private void print_limelight_results() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Result is null?", result == null);
        if (result != null) {
            telemetry.addData("Result valid?", result.isValid());
            telemetry.addData("Pipeline", result.getPipelineIndex());
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Botpose MT2 is null?", result.getBotpose_MT2() == null);
        }
    }

    private void initializeHopper() {
        //FIXME: DISTANCE SENSOR ISN'T WORKING, CODE NEEDS TUNING WHEN SENSOR IS FIXED
        double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
        int i = 0;
        while (distanceCm > 2 && i <= 1000) {
            hopper.setPower(-0.08);
            distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("HopperDistCM:", distanceCm);
            telemetry.update();
            i++;
        }
        hopper.setPower(0.0);
    }

    //Method for controlling the robot during the main loop
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

    private void driveRobot() {
        //limit motors to half speed
        double drive = gamepad1.left_stick_y * 0.5;
        double strafe = gamepad1.left_stick_x * 0.5;
        double turn = gamepad1.right_stick_x * 0.5;

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

    private boolean resetOdometry(boolean lastDownArrow) {
        //resetting odometry positions and heading
        boolean currDownArrow = gamepad1.dpad_down;
        if (currDownArrow && !lastDownArrow) {
            odometryComputer.resetPosAndIMU();
        }
        lastDownArrow = currDownArrow;
        return lastDownArrow;
    }

    private double shootBall(boolean ball_is_ready) {
        double hopperPower = 0;
        double shootPower_r = 0;
        double shootPower_l = 0;
        if (gamepad1.a) {
            shootPower_r = -0.6;
            shootPower_l = 0.6;
            if (!ball_is_ready) {
                hopperPower = -0.08;
            } else {
                flipper.setPosition(0.1);
                sleep(2000);
                flipper.setPosition(0.6);
                hopper.setPower(-0.12);
                sleep(500);
                hopper.setPower(0.0);
            }
        }
        shooter_right.setPower(shootPower_r);
        shooter_left.setPower(shootPower_l);
        return hopperPower;
    }

    private double controlIntake() {
        double hp = 0; //hopper power
        if (gamepad1.b) {
            intake.setPower(0.95); //turn on intake motor
            hp = -0.15; //set hopper motor power (negative spins clockwise)
        } else if (gamepad1.y) {
            intake.setPower(-0.95); //turn on intake motor
            hp = 0.15; //set hopper motor power (negative spins clockwise)
        } else {
            intake.setPower(0); //turn off intake motor
        }

        return hp;
    }
    private void controlFlipper() {
        // Lift ball into Shooter using Flipper
        if (gamepad1.dpad_up) {
            flipper.setPosition(0.1);
        } else {
            flipper.setPosition(0.6);
        }
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

    private String getBallColor(NormalizedColorSensor cSensor) {
        //this method returns ball color green, purple or unknown
        NormalizedRGBA colors = cSensor.getNormalizedColors();

        //convert RGBA values to HSV color wheel values for better color mapping
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        //return hsv values for testing color
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        //check values to determine color
        String returnColor;
        if (saturation < 0.2 || value < 0.05) {
            returnColor = "UNKNOWN";
        } else if (hue >= 90 && hue <= 185) {
            returnColor = "GREEN";
        } else if (hue >= 210 && hue <= 330) {
            returnColor = "PURPLE";
        } else {
            returnColor = "UNKNOWN";
        }

        //FIXME: THESE EXTRA OUTPUTS ARE ONLY FOR TESTING
        // DELETE EXTRA OUTPUTS AFTER CODE VERIFIED
        returnColor += String.format(" H: %.2f, S: %.2f, V: %.2f", hue, saturation, value) + ", ";

        return returnColor;
    }

    private boolean hood_down (boolean lastDownArrow) {
        //resetting odometry positions and heading
        boolean currDownArrow = (gamepad1.dpad_down) ;
        double currHoodPosition = hood.getPosition() ;
        if (currDownArrow && !lastDownArrow) {
            hood.setPosition(currHoodPosition - 0.1) ;
        }
        lastDownArrow = currDownArrow;
        telemetry.addData("hood pos",hood.getPosition());
        return lastDownArrow;
    }
    private boolean hood_up (boolean lastUpArrow) {
        //resetting odometry positions and heading
        boolean currUpArrow = (gamepad1.dpad_up) ;
        double currHoodPosition = hood.getPosition() ;
        if (currUpArrow && !lastUpArrow) {
            hood.setPosition(currHoodPosition + 0.1) ;
        }
        lastUpArrow = currUpArrow;
        telemetry.addData("hood pos",hood.getPosition());
        return lastUpArrow;
    }

}