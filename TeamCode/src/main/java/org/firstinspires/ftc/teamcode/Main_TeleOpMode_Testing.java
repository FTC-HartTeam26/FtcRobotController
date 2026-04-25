/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp(name="Testing Hardware v1", group="TeleOpModes")
public class Main_TeleOpMode_Testing extends LinearOpMode {
    double oldTime = 0; //used to calculate loop frequency

    @Override
    public void runOpMode() {
        //motor set up
        DcMotor back_right = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor shooter_right = hardwareMap.get(DcMotor.class, "shooter_right");
        DcMotor shooter_left = hardwareMap.get(DcMotor.class, "shooter_left");

        //servo setup
        CRServo hopper = hardwareMap.get(CRServo.class, "hopper");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");

        //sensor setup
        GoBildaPinpointDriver odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class,"odometry");

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

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drive
            front_right.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
            back_right.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
            front_left.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);
            back_left.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);

            //Turn on Shooting Motors
            if (gamepad1.a) {
                shooter_right.setPower(1);
                shooter_left.setPower(-1);
            } else {
                shooter_right.setPower(0);
                shooter_left.setPower(0);
            }

            //Turn on Intake Motors and spin Hopper
            if (gamepad1.b) {
                intake.setPower(1);
                hopper.setPower(-.15);
            } else {
                intake.setPower(0);
                hopper.setPower(0);
            }

            // Lift ball into Shooter using Flipper
            if (gamepad1.dpad_up) {
                flipper.setPosition(1.0);
            } else {
                flipper.setPosition(0.0);
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
