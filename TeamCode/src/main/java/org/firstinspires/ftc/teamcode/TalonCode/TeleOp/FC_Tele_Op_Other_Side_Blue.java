package org.firstinspires.ftc.teamcode.TalonCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.ErikCode.Testerino.Vuforia.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.TalonCode.HardwareMap.Zoinkifier;


@TeleOp(name = "Whack TeleOp Blue", group = "Zoinkifier")
public class FC_Tele_Op_Other_Side_Blue extends OpMode {

    Zoinkifier robot;

    //The angle that the robot should drive at relative to its starting position, Pi/2 corresponds to straight forward
    double angleFromDriver = Math.PI;
    boolean firstTime = true;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        robot = new Zoinkifier(hardwareMap, telemetry);
    }

    @Override
    public void loop (){
        if(firstTime) {
            robot.deployIntake();
            firstTime = false;
        }

        robot.updateGyro();
        telemetry.addData("Gyro Heading", Math.toDegrees(robot.heading));
        if(gamepad1.dpad_left)
            robot.currentDrivePower = .2;
        else
            robot.currentDrivePower = robot.DRIVE_POWER;

        //Changes the angle of the robot's motion with respect to the driver to its current angle when the start button is pressed on gamepad one
        if(gamepad1.y)
            angleFromDriver = robot.heading;

        if(gamepad1.x)
            robot.farOut();
        if(gamepad1.b)
            robot.deployIntake();

        //Runs the intake spinners with bumpers
        if(gamepad1.left_bumper)
            robot.runIntake(robot.INTAKE_POWER, robot.INTAKE_POWER);
        else if (gamepad1.right_bumper)
            robot.runIntake(- robot.INTAKE_POWER, - robot.INTAKE_POWER);
        else
            robot.runIntake(0.25 * (gamepad1.left_trigger - gamepad1.right_trigger), 0.25 * ( - gamepad1.left_trigger + gamepad1.right_trigger));

        //Dpad controls flipper
        if (gamepad1.dpad_up)
            robot.flipper.setPower(robot.FLIPPER_POWER);
        else if (gamepad1.dpad_down)
            robot.flipper.setPower(-robot.FLIPPER_POWER);
        else
            robot.flipper.setPower(0);

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

        robot.drive(
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.left_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.left_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.left_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.left_stick_x) * robot.currentDrivePower
        );

        telemetry.addData("Front Left Encoder", robot.fleft.getCurrentPosition());
        telemetry.addData("Front Right Encoder", robot.fright.getCurrentPosition());
        telemetry.addData("Back Left Encoder", robot.bleft.getCurrentPosition());
        telemetry.addData("Back Right Encoder", robot.bright.getCurrentPosition());
    }
}