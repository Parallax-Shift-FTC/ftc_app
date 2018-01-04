package org.firstinspires.ftc.teamcode.ErikCode.TeleOpTemplates.HolonomicOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
Tests the Dpad Functions moving rotating (just top and right as of now)
 */
@Disabled
@TeleOp(name = "Holonomic FieldCentric Tele-Op Test 4", group = "holonomic Erik")
public class Holonomic_FieldCentric_Erik_NewControls4 extends OpMode
{
    Holonomic_Hardware robot;
    double angleFromDriver = Math.PI;
    double jTheta;
    double jp;
    double theta;
    boolean robotCentric = false;
    boolean shouldTurn = false;
    double desiredHeading;
    double turnspeed;
    double error;
    double turnDirection;

    @Override
    public void init ()
    {
        robot = new Holonomic_Hardware(hardwareMap, telemetry, true);
    }

    @Override
    public void loop ()
    {
        robot.updateGyro();

        if(gamepad1.dpad_up && !shouldTurn)
        {
            shouldTurn = true;
            desiredHeading = 0;
        }

        if(gamepad1.dpad_down && !shouldTurn)
        {
            shouldTurn = true;
            desiredHeading = Math.PI;
        }

        if(gamepad1.dpad_left && !shouldTurn)
        {
            shouldTurn = true;
            desiredHeading = Math.PI/2;
        }

        if(gamepad1.dpad_right && !shouldTurn)
        {
            shouldTurn = true;
            desiredHeading = 3*Math.PI/2;
        }

        if(desiredHeading == 0 || desiredHeading == 2*Math.PI)
        {
            if(robot.heading > Math.PI)
            {
                desiredHeading = 2*Math.PI;
            }
            else
            {
                desiredHeading = 0;
            }
        }

        error = desiredHeading - robot.heading;
        turnDirection = Math.abs(error) / error;
        turnspeed = error/Math.PI;

        if(gamepad1.b) {
            shouldTurn = false;
        }

        jTheta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        jp = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);

        if(jp > 1)
            jp = 1;

        theta = (jTheta + angleFromDriver - robot.heading);

        if(!shouldTurn) {
            robot.drive(
                    (Math.sin(theta) + Math.cos(theta)) * jp / 2 - gamepad1.right_stick_x,
                    (Math.sin(theta) - Math.cos(theta)) * jp / 2 + gamepad1.right_stick_x,
                    (Math.sin(theta) - Math.cos(theta)) * jp / 2 - gamepad1.right_stick_x,
                    (Math.sin(theta) + Math.cos(theta)) * jp / 2 + gamepad1.right_stick_x
            );
        }
        else if(shouldTurn)
        {
            robot.drive(
                    (Math.sin(theta) + Math.cos(theta)) * jp / 2 - turnspeed,
                    (Math.sin(theta) - Math.cos(theta)) * jp / 2 + turnspeed,
                    (Math.sin(theta) - Math.cos(theta)) * jp / 2 - turnspeed,
                    (Math.sin(theta) + Math.cos(theta)) * jp / 2 + turnspeed
            );
        }

        telemetry.addData("Ultra Turbo Mode Activated", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.addData(" Right Joystick X Axis:", gamepad1.right_stick_x);
        telemetry.addData("Joystick Direction", Math.toDegrees(jTheta));
        telemetry.addData("Joystick Magnitude", jp);
        telemetry.addData("Gyro Heading", robot.heading);
        telemetry.addData("robotCentric", robotCentric);
        telemetry.addData("tophat up", gamepad1.dpad_up);
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Turn Speed", turnspeed);
        telemetry.addData("Should Turn", shouldTurn);
    }
}