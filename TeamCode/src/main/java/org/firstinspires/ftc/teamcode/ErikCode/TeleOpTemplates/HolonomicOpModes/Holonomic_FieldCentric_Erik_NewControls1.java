package org.firstinspires.ftc.teamcode.ErikCode.TeleOpTemplates.HolonomicOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
Program where using the bumpers simply takes you to the last gyro heading retrieved
 */
@Disabled
@TeleOp(name = "Holonomic FieldCentric Tele-Op Test 1", group = "holonomic Erik")
public class Holonomic_FieldCentric_Erik_NewControls1 extends OpMode
{
    Holonomic_Hardware robot;
    double angleFromDriver = Math.PI;
    double jTheta;
    double jp;
    double theta;
    boolean robotCentric = false;
    boolean lobotCentric = false;
    int dpad = 0;
    int z = 1;

    @Override
    public void init ()
    {
        robot = new Holonomic_Hardware(hardwareMap, telemetry, true);
    }

    @Override
    public void loop ()
    {
        if(gamepad1.right_bumper)
            robotCentric = true;
        else
            robotCentric = false;

        if(gamepad1.left_bumper)
            lobotCentric = true;
        else
            lobotCentric = false;

        //Makes it so it becomes robot centric based on the last heading before pressing the button
        if(robotCentric == false)
            robot.updateGyro();

        jTheta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        jp = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);

        if(jp > 1)
            jp = 1;

        if(lobotCentric == false)
            theta = (jTheta + angleFromDriver - robot.heading);
        else if(lobotCentric)
            theta = jTheta;

        robot.drive(
                 (Math.sin(theta)+Math.cos(theta))*jp/2 - gamepad1.right_stick_x,
                (Math.sin(theta)-Math.cos(theta))*jp/2 + gamepad1.right_stick_x,
                (Math.sin(theta)-Math.cos(theta))*jp/2 - gamepad1.right_stick_x,
                (Math.sin(theta)+Math.cos(theta))*jp/2 + gamepad1.right_stick_x
        );

        telemetry.addData("Ultra Turbo Mode Activated", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.addData(" Right Joystick X Axis:", gamepad1.right_stick_x);
        telemetry.addData("Joystick Direction", Math.toDegrees(jTheta));
        telemetry.addData("Joystick Magnitude", jp);
        telemetry.addData("Gyro Heading", robot.heading);
        telemetry.addData("robotCentric", robotCentric);


    }
}