package org.firstinspires.ftc.teamcode.Tournament.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;

@Autonomous(name = "Far Blue Auto Left", group = "Autonomous")
public class Far_Blue_Left extends LinearOpMode {

    Zoinkifier robot;
    int strafeDistance = robot.FAR_STONE_CLOSE_SLOT;

    @Override
    public void runOpMode() {
        //Initialization
        telemetry.addData("Ready to begin", false);
        telemetry.update();

        robot = new Zoinkifier(hardwareMap);

        //Resets the encoders
        robot.fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the correct modes for the motors
        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initializes the jewel arm servos
        robot.bottomServo.setPosition(0.03);
        robot.topServo.setPosition(0.25);

        telemetry.addData("Ready to begin", true);
        telemetry.update();
        waitForStart();

        //Puts the intake in its starting position
        robot.deployIntake();

        //Moves the sensor into position and takes a reading from the color sensor, then hits the
        //correct colored jewel and folds the jewel arm back up
        robot.bottomServo.setPosition(0.55);
        robot.topServo.setPosition(0.75);
        sleep(200);
        robot.bottomServo.setPosition(0.68);
        sleep(500);
        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() > robot.colorSensor.blue())
            robot.topServo.setPosition(.60);
        else if(robot.colorSensor.red() < robot.colorSensor.blue())
            robot.topServo.setPosition(.86);
        sleep(1000);
        robot.bottomServo.setPosition(0.55);
        sleep(200);
        robot.topServo.setPosition(0.25);
        robot.bottomServo.setPosition(0.03);
        sleep(500);

        //Starts driving, then uses the rotation about the y axis to tell when the robot has gotten
        //off the balancing stone and stops
        robot.drive(.2,.2,.2,.2);
        telemetry.addData("State", "Drive until Tilted");
        telemetry.update();
        do {
            robot.updateGyro();
            idle();
        }
        while(robot.yRotation > Math.toRadians(-2) && opModeIsActive());
        telemetry.addData("State", "Drive until not Tilted");
        telemetry.update();
        sleep(250);
        do {
            robot.updateGyro();
            idle();
        }
        while(robot.yRotation < Math.toRadians(-2) && opModeIsActive());
        robot.brake();

        //Drives forward using the encoders
        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        robot.setDriveEncoders(.4,.4,.4,.4, 500,500,500,500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        //Turns 180 degrees
        telemetry.addData("State", "Do a 180");
        telemetry.update();
        robot.updateGyro();
        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.updateGyro();
        double error = robot.heading + Math.PI / 2;
        while(error > Math.toRadians(6) && opModeIsActive()) {
            robot.updateGyro();
            error = robot.heading + Math.PI / 2;
            robot.drive( .1 + error * 0.4 / Math.PI, - .1 - error * 0.4 / Math.PI,  .1 + error * 0.4 / Math.PI, - .1 - error * 0.4 / Math.PI);
            idle();
        }
        robot.brake();

        //Drives forward to get close to the cryptobox
        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        //Drives forward using the encoders
        robot.setDriveEncoders(-.4,-.4,-.4,-.4, -600,-600,-600,-600);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        //Strafes sideways the correct amount to line up with the correct slot
        telemetry.addData("State", "Drive Sideways With Encoders");
        telemetry.update();
        //Drives to the correct place using the encoders
        robot.setDriveEncoders(-.2, .2, .2, -.2, -strafeDistance, strafeDistance, strafeDistance, -strafeDistance);
        while (robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        //Flips the cube up
        robot.flipper.setTargetPosition(500);
        robot.flipper.setPower(robot.FLIPPER_POWER * 0.75);
        while(robot.flipper.isBusy() && opModeIsActive())
            idle();
        robot.flipper.setPower(0);

        //Drives back, goes forward again to hit the cube in, then drives back again to park
        robot.setDriveEncoders(.2,.2,.2,.2, 500, 500, 500, 500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.setDriveEncoders(-.2,-.2,-.2,-.2, -600, -600, -600, -600);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.setDriveEncoders(.2,.2,.2,.2, 500, 500, 500, 500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        //Brings the flipper back down
        robot.flipper.setTargetPosition(0);
        robot.flipper.setPower(-robot.FLIPPER_POWER * 1/2);
        while(robot.flipper.isBusy() && opModeIsActive())
            idle();
        robot.flipper.setPower(0);
    }
}
