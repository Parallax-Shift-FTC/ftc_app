package org.firstinspires.ftc.teamcode.TalonCode.DaquanOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Talon's Field Centric", group = "Daquan")
public class Fancy_Field_Centric extends OpMode {

    Daquan_Hardware robot;

    //Sets up states for the state machine
    int currentState = 0;
    static final int FIELD_CENTRIC_STATE = 0;
    static final int ROBOT_CENTRIC_STATE = 1;
    static final int DPAD_STATE = 2;

    //Target angle for turning
    double targetAngle;
    double turningSpeed;

    //The angle that the robot should drive at relative to its starting position, Pi/2 corresponds to straight forward
    static final double ANGLE_FROM_DRIVER = Math.PI / 2;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        robot = new Daquan_Hardware(hardwareMap, telemetry, true);
    }

    @Override
    public void loop (){

        //Moves the intake wheels based on the left joystick
        //robot.rurricane.setPower(- gamepad1.left_stick_y * INTAKE_POWER);
        //robot.lurricane.setPower(- gamepad1.left_stick_y * INTAKE_POWER);

        robot.updateGyro();
        telemetry.addData("Gyro Heading", Math.toDegrees(robot.heading));

        //State machine
        switch(currentState) {

            case FIELD_CENTRIC_STATE:
                //This is the angle that the right joystick is pointing in
                double inputAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
                telemetry.addData("Joystick Direction", Math.toDegrees(inputAngle));

                //This is the magnitude of how far the joystick is pushed
                double inputPower = Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);
                telemetry.addData("Joystick Magnitude", inputPower);

                if (inputPower > 1)
                    inputPower = 1;

                //This is the angle at which the robot should translate
                double moveAngle = inputAngle + (ANGLE_FROM_DRIVER - robot.heading);

                robot.drive(
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower / 2 + gamepad1.right_trigger - gamepad1.left_trigger,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower / 2 - gamepad1.right_trigger + gamepad1.left_trigger,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower / 2 + gamepad1.right_trigger - gamepad1.left_trigger,
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower / 2 - gamepad1.right_trigger + gamepad1.left_trigger
                );

                //If the dpad is pressed it will go to the turning state and set the target angle
                if(gamepad1.dpad_up) {
                    targetAngle = Math.PI / 2;
                    currentState = DPAD_STATE;
                }
                else if(gamepad1.dpad_right) {
                    targetAngle = 0;
                    currentState = DPAD_STATE;
                }
                else if(gamepad1.dpad_down) {
                    targetAngle = - Math.PI / 2;
                    currentState = DPAD_STATE;
                }
                else if(gamepad1.dpad_left) {
                    targetAngle = Math.PI;
                    currentState = DPAD_STATE;
                }
                //If the right bumper is held it will go to the robot centric state
                else if(gamepad1.right_bumper)
                    currentState = ROBOT_CENTRIC_STATE;

                break;

            case ROBOT_CENTRIC_STATE:
                //Arcade drive with right joystick, turn with triggers(clockwise-right, counterclockwise-left)
                //Format: +/- Turning +/- Forward/Backward +/- Strafing
                robot.drive(
                        clipValue(robot.DRIVE_POWER * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.DRIVE_POWER * (- gamepad1.right_stick_y) + robot.DRIVE_POWER * (gamepad1.right_stick_x)),
                        clipValue(robot.DRIVE_POWER * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.DRIVE_POWER * (- gamepad1.right_stick_y) + robot.DRIVE_POWER * (- gamepad1.right_stick_x)),
                        clipValue(robot.DRIVE_POWER * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.DRIVE_POWER * (- gamepad1.right_stick_y) + robot.DRIVE_POWER * (- gamepad1.right_stick_x)),
                        clipValue(robot.DRIVE_POWER * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.DRIVE_POWER * (- gamepad1.right_stick_y) + robot.DRIVE_POWER * (gamepad1.right_stick_x))
                );

                //If the dpad is pressed it will go to the turning state and set the target angle
                if(gamepad1.dpad_up) {
                    targetAngle = Math.PI / 2;
                    currentState = DPAD_STATE;
                }
                else if(gamepad1.dpad_right) {
                    targetAngle = 0;
                    currentState = DPAD_STATE;
                }
                else if(gamepad1.dpad_down) {
                    targetAngle = - Math.PI / 2;
                    currentState = DPAD_STATE;
                }
                else if(gamepad1.dpad_left) {
                    targetAngle = Math.PI;
                    currentState = DPAD_STATE;
                }
                    //If the right bumper is no longer held down it will go to the field centric state
                else if(gamepad1.right_bumper)
                    currentState = ROBOT_CENTRIC_STATE;

                break;

            case DPAD_STATE:
                //Slows the turning once teh robot is within 5 degrees of its target
                if(Math.abs(robot.heading - targetAngle) > Math.toRadians(5))
                    turningSpeed = 1;
                else
                    turningSpeed = 0.2;

                //Turns the robot if it isn't within .5 degrees of its target angle, otherwise sets the state back to field centric
                if(robot.heading > targetAngle + Math.toRadians(.5) || robot.heading < targetAngle - Math.toRadians(.5))
                    robot.turn(Math.abs(targetAngle - robot.heading) / (targetAngle - robot.heading), turningSpeed);
                else
                    currentState = FIELD_CENTRIC_STATE;

                //If B is pressed it will go back to the field centric state
                if (gamepad1.b)
                    currentState = FIELD_CENTRIC_STATE;

                break;
        }
    }

    //Clips the values given to the motors so that they don't go over 1
    double clipValue(double value) {
        if(value > robot.DRIVE_POWER || value < - robot.DRIVE_POWER)
            return(value / Math.abs(value) * robot.DRIVE_POWER);
        else
            return value;
    }
}