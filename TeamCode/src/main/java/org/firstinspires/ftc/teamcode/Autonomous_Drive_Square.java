package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto_Drive_Square", group="AutoMode")
public class Autonomous_Drive_Square extends LinearOpMode {
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_right;
    private GoBildaPinpointDriver odometryComputer;

    @Override

    public void runOpMode() {

        initializeHardware();
        configureOdometry();
        configureDriveMotors();

        // Wait for the game to start (driver presses START)
        waitForStart();

        driveToPosition_Relative(304.8); // using MM, i.e. 12 inches = 304.8 mm

        rotate_Relative(90);
        driveToPosition_Relative(304.8);

        rotate_Relative(90);
        driveToPosition_Relative(304.8);

        rotate_Relative(90);
        driveToPosition_Relative(304.8);

    }
    private void driveToPosition_Relative(double dist) {
        double margin_of_error = 5.0; // update error margin as needed
        double drivePower = 0.5; // update drive power as needed

        // Record exact starting coordinates
        double startX = odometryComputer.getPosX(DistanceUnit.MM);
        double startY = odometryComputer.getPosY(DistanceUnit.MM);
        double distanceTraveled = 0;

        // Loop until the straight-line distance matches the target
        while (opModeIsActive() &&
                distanceTraveled < (dist - margin_of_error)) {
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
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower));

            // Add minimum power so the robot doesn't stall near the target
            if (turnPower > 0 && turnPower < minPower) turnPower = minPower;
            if (turnPower < 0 && turnPower > -minPower) turnPower = -minPower;

            driveRobot(0, 0, -turnPower);
        }
        driveRobot(0, 0, 0); // Stop the robot
    }
    private void initializeHardware(){
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
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
}
