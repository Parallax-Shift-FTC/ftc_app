package org.firstinspires.ftc.teamcode.TalonCode.DaquanOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Talon's Fancy Field Centric", group = "Daquan")
public class Fancy_Field_Centric extends OpMode {

    Daquan_Hardware robot;

    //Sets up states for the state machine
    int currentState = 0;
    static final int FIELD_CENTRIC_STATE = 0;
    static final int ROBOT_CENTRIC_STATE = 1;
    static final int TURNING_STATE = 2;

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

        //Ultra turbo and sneak modes
        if(- gamepad1.left_stick_y > 0)
            robot.currentDrivePower = .8;
        else if(- gamepad1.left_stick_y < 0)
            robot.currentDrivePower = .1;
        else
            robot.currentDrivePower = robot.DRIVE_POWER;

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
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + gamepad1.right_trigger - gamepad1.left_trigger,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower - gamepad1.right_trigger + gamepad1.left_trigger,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + gamepad1.right_trigger - gamepad1.left_trigger,
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower - gamepad1.right_trigger + gamepad1.left_trigger
                );

                //If the dpad is pressed it will go to the turning state and set the target angle
                if(gamepad1.dpad_up) {
                    targetAngle = Math.PI / 2;
                    currentState = TURNING_STATE;
                }
                else if(gamepad1.dpad_right) {
                    targetAngle = 0;
                    currentState = TURNING_STATE;
                }
                else if(gamepad1.dpad_down) {
                    targetAngle = - Math.PI / 2;
                    currentState = TURNING_STATE;
                }
                else if(gamepad1.dpad_left) {
                    targetAngle = Math.PI;
                    currentState = TURNING_STATE;
                }
                //If start is pressed it goes to the "start state", which turns the robot's front to face its current direction of motion;
                else if(gamepad1.start) {
                    targetAngle = moveAngle;
                    currentState = TURNING_STATE;
                }
                //If the right bumper is held it will go to the robot centric state
                else if(gamepad1.right_bumper)
                    currentState = ROBOT_CENTRIC_STATE;

                break;

            case ROBOT_CENTRIC_STATE:
                //Arcade drive with right joystick, turn with triggers(clockwise-right, counterclockwise-left)
                //Format: +/- Turning +/- Forward/Backward +/- Strafing
                robot.drive(
                        clipValue(robot.currentDrivePower * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x)),
                        clipValue(robot.currentDrivePower * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x)),
                        clipValue(robot.currentDrivePower * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x)),
                        clipValue(robot.currentDrivePower * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x))
                );

                //If the dpad is pressed it will go to the turning state and set the target angle
                if(gamepad1.dpad_up) {
                    targetAngle = Math.PI / 2;
                    currentState = TURNING_STATE;
                }
                else if(gamepad1.dpad_right) {
                    targetAngle = 0;
                    currentState = TURNING_STATE;
                }
                else if(gamepad1.dpad_down) {
                    targetAngle = - Math.PI / 2;
                    currentState = TURNING_STATE;
                }
                else if(gamepad1.dpad_left) {
                    targetAngle = Math.PI;
                    currentState = TURNING_STATE;
                }
                    //If the right bumper is no longer held down it will go to the field centric state
                else if(!gamepad1.right_bumper)
                    currentState = FIELD_CENTRIC_STATE;

                break;

            case TURNING_STATE:

                //Add target angle to telemetry fo debugging
                telemetry.addData("Target Angle", Math.toDegrees(targetAngle));
                //This is the angle that the right joystick is pointing in
                inputAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
                telemetry.addData("Joystick Direction", Math.toDegrees(inputAngle));

                //This is the magnitude of how far the joystick is pushed
                inputPower = Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);
                telemetry.addData("Joystick Magnitude", inputPower);

                if (inputPower > 1)
                    inputPower = 1;

                //This is the angle at which the robot should translate
                moveAngle = inputAngle + (ANGLE_FROM_DRIVER - robot.heading);

                //If the robot isn't within 1 degree of the target angle, it drives in a field centric fashion while turning
                if(Math.abs(robot.heading - targetAngle) > Math.toRadians(1)) {

                    //Sets the turning speed and direction based on how close the robot is to the target angle
                    if (Math.abs(robot.heading - targetAngle) > Math.toRadians(30))
                        turningSpeed = .2 * Math.abs(targetAngle - robot.heading) / (targetAngle - robot.heading);
                    else
                        turningSpeed = 0.1 * Math.abs(targetAngle - robot.heading) / (targetAngle - robot.heading);
                    telemetry.addData("Target Angle", targetAngle);

                    robot.drive(
                            (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower - turningSpeed,
                            (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + turningSpeed,
                            (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower - turningSpeed,
                            (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + turningSpeed
                    );
                }
                else {
                    robot.brake();
                    currentState = FIELD_CENTRIC_STATE;
                }


                //Exits out of turning if b is pressed
                if(gamepad1.b) {
                    robot.brake();
                    currentState = FIELD_CENTRIC_STATE;
                }

                break;
        }

        //Tells the driver the current state
        if(currentState == FIELD_CENTRIC_STATE)
            telemetry.addData("State", "Field Centric");
        else if(currentState == ROBOT_CENTRIC_STATE)
            telemetry.addData("State", "Robot Centric");
        else if(currentState == TURNING_STATE)
            telemetry.addData("State", "Turning");
        else
            telemetry.addData("State", "???");
}
    //Clips the values given to the motors so that they don't go over 1
    double clipValue(double value) {
        if(value > robot.DRIVE_POWER || value < - robot.DRIVE_POWER)
            return(value / Math.abs(value) * robot.DRIVE_POWER);
        else
            return value;
    }
}