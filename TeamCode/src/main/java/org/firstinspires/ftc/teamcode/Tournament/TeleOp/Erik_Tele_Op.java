package org.firstinspires.ftc.teamcode.Tournament.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;
@Disabled
@TeleOp(name = "Erik TeleOp", group = "Zoinkifier")
public class Erik_Tele_Op extends OpMode {

    Zoinkifier robot;

    //The angle that the robot should drive at relative to its starting position, PI/2 corresponds to straight forward
    double angleFromDriver = Math.PI / 2;
    boolean firstTime = true;
    boolean fieldCentric = true;
    boolean usingDpad = false;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        telemetry.addData("Ready to begin", false);
        telemetry.update();
        robot = new Zoinkifier(hardwareMap);
        telemetry.addData("Ready to begin", true);
        telemetry.update();
    }

    @Override
    public void loop () {
        //Initializes servos here because we can't do it in initialization
        if (firstTime) {
            robot.deployIntake();
            robot.bottomServo.setPosition(0);
            robot.topServo.setPosition(0.25);
            firstTime = false;
        }

        //Puts the robot in slow mode while the left dpad button is held
        if ((gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down) && !usingDpad) {
            usingDpad = true;
            robot.currentDrivePower = robot.SLOW_POWER;
            robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (!(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down) && usingDpad) {
            usingDpad = false;
            robot.currentDrivePower = robot.DRIVE_POWER;
            robot.fleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.bleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.bright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Moves the intake wheels out further when x is pressed and back to normal when b is pressed
        if (gamepad1.y)
            robot.farOut();
        if (gamepad1.x)
            robot.deployIntake();

        //Runs the intake spinners in when the left bumper is pressed and out when the right one is
        //pressed
        if (gamepad1.right_bumper)
            robot.runIntake(robot.INTAKE_POWER, robot.INTAKE_POWER);
        else if (gamepad1.left_bumper)
            robot.runIntake(-robot.INTAKE_POWER, -robot.INTAKE_POWER);
        else
            robot.runIntake(0.25 * (gamepad1.left_trigger - gamepad1.right_trigger), 0.25 * (-gamepad1.left_trigger + gamepad1.right_trigger));

        //Y and A control the flipper
        if (gamepad1.a)
            robot.flipper.setPower(robot.FLIPPER_POWER);
        else if (gamepad1.b)
            robot.flipper.setPower(-robot.FLIPPER_POWER);
        else
            robot.flipper.setPower(0);

        //Updates the gyro sensor for field centric drive
        robot.updateGyro();
        telemetry.addData("Gyro Heading", Math.toDegrees(robot.heading));

        //Changes the angle of the robot's motion with respect to the driver to the robot's current
        //heading when the start is pressed, and also makes the robot drive in a field centric
        //manner. Back makes it robot centric
        if (gamepad1.start) {
            fieldCentric = true;
            angleFromDriver = robot.heading;
        }
        else if (gamepad1.back) {
            fieldCentric = false;
        }
        telemetry.addData("Field Centric", fieldCentric);

        //This is the angle that the right joystick is pointing in, or the angle assigned by the dpad
        double inputAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        if(usingDpad) {
            if(gamepad1.dpad_right)
                inputAngle = 0;
            if(gamepad1.dpad_up)
                inputAngle = Math.PI / 2;
            if(gamepad1.dpad_left)
                inputAngle = Math.PI;
            if(gamepad1.dpad_down)
                inputAngle = - Math.PI / 2;
        }

        //This is the magnitude of how far the joystick is pushed (automatically 1 for the dpad)
        double inputPower = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);
        if (inputPower > 1 || usingDpad)
            inputPower = 1;

        //This is the angle at which the robot should translate
        double moveAngle = inputAngle + (angleFromDriver - robot.heading);
        if(!fieldCentric)
            moveAngle = inputAngle;

        //Assigns the values to the wheels
        robot.drive(
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.right_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (-gamepad1.right_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.right_stick_x) * robot.currentDrivePower,
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (-gamepad1.right_stick_x) * robot.currentDrivePower
        );
    }
}