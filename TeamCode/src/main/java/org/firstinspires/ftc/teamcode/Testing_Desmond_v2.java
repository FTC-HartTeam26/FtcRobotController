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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Testing_Desmond_v2", group="TeleOpModes")
public class Testing_Desmond_v2 extends LinearOpMode {
    private Limelight3A limelight;
    private double distance = 0;
    private IMU imu;

    @Override
    public void runOpMode() {
        //motor set up
        DcMotor back_right = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor back_left = hardwareMap.get(DcMotor.class, "back_left");

        //set up drive motor directions
        back_right.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);

        //limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        imu.resetYaw();
        limelight.start();

        //Display initial limelight status
        telemetry.addData("Status", "Limelight initialized");
        telemetry.addData("Limelight connected?", limelight.isConnected());
        telemetry.addData("Limelight running?", limelight.isRunning());
        telemetry.addData("Time since update", limelight.getTimeSinceLastUpdate());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drive
            front_right.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
            back_right.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) + gamepad1.right_stick_x * 0.5);
            front_left.setPower((gamepad1.left_stick_y * 0.5 - gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);
            back_left.setPower((gamepad1.left_stick_y * 0.5 + gamepad1.left_stick_x * 0.5) - gamepad1.right_stick_x * 0.5);

            //Get data from Limelight
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult llResult = limelight.getLatestResult();

            //Add limelight results to telemetry for debugging
            telemetry.addData("Limelight result is null?", llResult == null);
            if (llResult != null) {
                telemetry.addData("Limelight result valid?", llResult.isValid());
                telemetry.addData("Pipeline", llResult.getPipelineIndex());
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                if (llResult.isValid()) {
                    distance = getDistance(llResult.getTy());
                    telemetry.addData("Distance", distance);
                    telemetry.addData("BotPose is null?", llResult.getBotpose_MT2() == null);
                } else {
                    telemetry.addData("Limelight", "Result returned, but no valid target");
                }
            }

            //Output telemetry
            telemetry.update();
        }
    }

    public double getDistance(double ty){
        //limelight distance calculations
        double CAMERA_ANGLE = 20;
        double angleToTarget = CAMERA_ANGLE + ty;
        double CAMERA_HEIGHT_IN = 12.625;
        double GOAL_HEIGHT = 29.5;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_IN;
        return  heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }
}
