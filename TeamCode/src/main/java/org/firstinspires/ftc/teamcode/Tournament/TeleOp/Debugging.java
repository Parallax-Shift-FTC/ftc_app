package org.firstinspires.ftc.teamcode.Tournament.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;

@TeleOp(name = "Debugging", group = "Zoinkifier")
public class Debugging extends OpMode {

        Zoinkifier robot;
        double angleFromDriver = Math.PI / 2;

        double leftServo = .8;
        double rightServo = .8;

        boolean lleft = false;
        boolean lright = false;
        boolean rleft = false;
        boolean rright = false;

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
            robot.leftIntakeArm.setPosition(leftServo);
            robot.rightIntakeArm.setPosition(rightServo);

            telemetry.addData("left servo position", leftServo);
            telemetry.addData("right servo position", rightServo);

            if(gamepad1.dpad_left && !lleft) {
                lleft = true;
                leftServo = leftServo + 0.005;
            }
            else
                lleft = false;

            if(gamepad1.dpad_right && !lright) {
                lright = true;
                leftServo = leftServo - 0.005;
            }
            else
                lright = false;

            if(gamepad1.x && !rleft) {
                rleft = true;
                rightServo = rightServo - 0.005;
            }
            else
                rleft = false;

            if(gamepad1.b && !rright) {
                rright = true;
                rightServo = rightServo + 0.005;
            }
            else
                rright = false;

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
            telemetry.addData("Flipper Encoder", robot.flipper.getCurrentPosition());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.addData("Red", robot.colorSensor.red());
        }
}
