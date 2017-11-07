package org.firstinspires.ftc.teamcode.TalonCode.DaquanOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
- Name: Daquan Robot-Centric Tele-Op with Talon's Controls
- Creator[s]: Erik
- Date Created: 10/7/17
- Objective: To drive our mecanum drive robot and us the intake
- Controls: The right joystick controls translational movement, while the triggers control
            rotation. Also, holding both bumpers sends the robot into ultra-turbo mode, which
            enhances its speed. Using the triggers on the second gamepad, you can move the hurricane
            intakes, with left being intake and right being outtake.
- Sensor Usage: None
- Key Algorithms: TBD
- Uniqueness: This program was designed to drive, and test our intake.  While not too unique, it
              does have interesting code regarding the mecanum wheels.
- Possible Improvements: Creating a version of the program that successfully incorporates field
centric Drive.
 */

@TeleOp(name = "Talon's Robot Centric", group = "Daquan")
public class Robot_Centric extends OpMode
{
    Daquan_Hardware robot;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init()
    {
        robot = new Daquan_Hardware(hardwareMap, telemetry, false);
    }

    @Override
    public void loop () {

        //Ultra turbo and sneak modes
        if(gamepad1.left_stick_x > 0)
            robot.currentDrivePower = robot.DRIVE_POWER + (1 - robot.DRIVE_POWER) * gamepad1.left_stick_x;
        else if(gamepad1.left_stick_x < 0)
            robot.currentDrivePower = robot.DRIVE_POWER - (robot.DRIVE_POWER - .2) * - gamepad1.left_stick_x;
        else
            robot.currentDrivePower = robot.DRIVE_POWER;

        //Arcade drive with right joystick, turn with triggers(clockwise-right, counterclockwise-left)
        //Format: +/- Turning +/- Forward/Backward +/- Strafing
        robot.drive(
                robot.currentDrivePower * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x),
                robot.currentDrivePower * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x),
                robot.currentDrivePower * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x),
                robot.currentDrivePower * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x)
        );
    }
}
