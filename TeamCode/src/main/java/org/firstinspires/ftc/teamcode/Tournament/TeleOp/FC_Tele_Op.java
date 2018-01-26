package org.firstinspires.ftc.teamcode.Tournament.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;

@TeleOp(name = "TeleOp", group = "Zoinkifier")
public class FC_Tele_Op extends OpMode {

    Zoinkifier robot;

    //The angle that the robot should drive at relative to its starting position, PI/2 corresponds to straight forward
    double angleFromDriver = Math.PI / 2;
    boolean firstTime = true;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        telemetry.addData("Ready to begin", false);
        telemetry.update();
        robot = new Zoinkifier(hardwareMap);
        telemetry.addData("Ready to begin", true);
        telemetry.update();
    }

    @Override
    public void loop (){
        //Initializes servos here because we can't do it in initialization
        if(firstTime) {
            robot.deployIntake();
            robot.bottomServo.setPosition(0);
            robot.topServo.setPosition(0.25);
            firstTime = false;
        }

        //Puts the robot in slow mode while the left dpad button is held
        if(gamepad1.dpad_left && robot.currentDrivePower != robot.SLOW_POWER) {
            robot.currentDrivePower = robot.SLOW_POWER;
            robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(!gamepad1.dpad_left && robot.currentDrivePower == robot.SLOW_POWER) {
            robot.currentDrivePower = robot.DRIVE_POWER;
            robot.fleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.bleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.bright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Moves the intake wheels out further when x is pressed and back to normal when b is pressed
        if(gamepad1.x)
            robot.farOut();
        if(gamepad1.b)
            robot.deployIntake();

        //Runs the intake spinners in when the left bumper is pressed and out when the right one is
        //pressed
        if(gamepad1.left_bumper)
            robot.runIntake(robot.INTAKE_POWER, robot.INTAKE_POWER);
        else if (gamepad1.right_bumper)
            robot.runIntake(- robot.INTAKE_POWER, - robot.INTAKE_POWER);
        else
            robot.runIntake(0.25 * (gamepad1.left_trigger - gamepad1.right_trigger), 0.25 * ( - gamepad1.left_trigger + gamepad1.right_trigger));

        //Dpad up and down control the flipper
        if (gamepad1.dpad_up)
            robot.flipper.setPower(robot.FLIPPER_POWER);
        else if (gamepad1.dpad_down)
            robot.flipper.setPower(-robot.FLIPPER_POWER);
        else if(gamepad1.dpad_right)
            robot.flipper.setPower(1);
        else
            robot.flipper.setPower(0);

        //Updates the gyro sensor for field centric drive
        robot.updateGyro();
        telemetry.addData("Gyro Heading", Math.toDegrees(robot.heading));

        //Changes the angle of the robot's motion with respect to the driver to the robot's current
        //heading when the Y button is pressed
        if(gamepad1.y)
            angleFromDriver = robot.heading;

        //This is the angle that the right joystick is pointing in
        double inputAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        telemetry.addData("Joystick Direction", Math.toDegrees(inputAngle));

        //This is the magnitude of how far the joystick is pushed
        double inputPower = Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);
        telemetry.addData("Joystick Magnitude", inputPower);
        if (inputPower > 1)
            inputPower = 1;

        //This is the angle at which the robot should translate
        double moveAngle = inputAngle + (angleFromDriver - robot.heading);

        //Drives the robot in a field centric fashion
        robot.drive(
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.left_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.left_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.left_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.left_stick_x) * robot.currentDrivePower
        );

        //Adds telemetry to help troubleshoot
        telemetry.addData("Front Left Encoder", robot.fleft.getCurrentPosition());
        telemetry.addData("Front Right Encoder", robot.fright.getCurrentPosition());
        telemetry.addData("Back Left Encoder", robot.bleft.getCurrentPosition());
        telemetry.addData("Back Right Encoder", robot.bright.getCurrentPosition());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.addData("Red", robot.colorSensor.red());
    }
}