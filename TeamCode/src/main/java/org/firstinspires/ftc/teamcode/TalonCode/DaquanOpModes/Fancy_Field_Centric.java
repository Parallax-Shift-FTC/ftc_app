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
    static final int DPAD_STATE = 2;

    //Target angle for turning
    double targetAngle;
    double turningSpeed;

    //The angle that the robot should drive at relative to its starting position, Pi/2 corresponds to straight forward
    double angleFromDriver = Math.PI / 2;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        robot = new Daquan_Hardware(hardwareMap, telemetry, true);
    }

    @Override
    public void loop (){

        robot.updateGyro();
        telemetry.addData("Gyro Heading", Math.toDegrees(robot.heading));

        telemetry.addData("Fleft Encoder", robot.fleft.getCurrentPosition());
        telemetry.addData("Fright Encoder", robot.fright.getCurrentPosition());
        telemetry.addData("Bleft Encoder", robot.bleft.getCurrentPosition());
        telemetry.addData("Bright Encoder", robot.bright.getCurrentPosition());

        //Ultra turbo and sneak modes
        if(gamepad1.left_stick_x > 0)
            robot.currentDrivePower = robot.DRIVE_POWER + (robot.maxPower - robot.DRIVE_POWER) * gamepad1.left_stick_x;
        else if(gamepad1.left_stick_x < 0)
            robot.currentDrivePower = robot.DRIVE_POWER - (robot.DRIVE_POWER - robot.minPower) * - gamepad1.left_stick_x;
        else
            robot.currentDrivePower = robot.DRIVE_POWER;

        if(gamepad1.y)
            angleFromDriver = robot.heading;

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
                double moveAngle = inputAngle + (angleFromDriver - robot.heading);

                robot.drive(
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.right_trigger - gamepad1.left_trigger) * robot.currentDrivePower,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.right_trigger + gamepad1.left_trigger) * robot.currentDrivePower,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.right_trigger - gamepad1.left_trigger) * robot.currentDrivePower,
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.right_trigger + gamepad1.left_trigger) * robot.currentDrivePower
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
                        robot.currentDrivePower * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x),
                        robot.currentDrivePower * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x),
                        robot.currentDrivePower * (gamepad1.right_trigger - gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x),
                        robot.currentDrivePower * (- gamepad1.right_trigger + gamepad1.left_trigger) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x)
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
                else if(!gamepad1.right_bumper)
                    currentState = FIELD_CENTRIC_STATE;

                break;

            case DPAD_STATE:

                //Add target angle to telemetry for debugging
                telemetry.addData("Target Angle", Math.toDegrees(targetAngle));
                
                //Changes the angle if turning to Pi so it functions correctly
                if(targetAngle == Math.PI && robot.heading < 0)
                    robot.heading = robot.heading + 2 * Math.PI;
                    
                double angleDifference = Math.abs(robot.heading - targetAngle);
                
                double turnDirection = angleDifference / (targetAngle - robot.heading);
                
                //If the robot isn't within 1 degree of the target angle, it drives in a field centric fashion while turning
                if(angleDifference > Math.toRadians(1)) {

                    //Sets the turning speed and direction based on how close the robot is to the target angle
                    if (angleDifference > Math.toRadians(30))
                        turningSpeed = .4 * turnDirection;
                    //Makes the robot turning change based on the square of the ratio of its current position to its desired position
                    else
                        turningSpeed = (0.1 + .2 * angleDifference * angleDifference  / (Math.toRadians(30) * Math.toRadians(30))) * turnDirection;
                    telemetry.addData("Target Angle", Math.toDegrees(targetAngle));
                    telemetry.addData("Turning Speed", turningSpeed * turnDirection);

                    robot.drive(
                            - turningSpeed,
                            turningSpeed,
                            - turningSpeed,
                            turningSpeed
                    );
                }
                else
                    robot.brake();
                //Exits out of turning if b is pressed
                if(gamepad1.b || gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_trigger !=0 || gamepad1.left_trigger !=0)
                    currentState = FIELD_CENTRIC_STATE;

                break;
        }

        //Tells the driver the current state
        if(currentState == FIELD_CENTRIC_STATE)
            telemetry.addData("State", "Field Centric");
        else if(currentState == ROBOT_CENTRIC_STATE)
            telemetry.addData("State", "Robot Centric");
        else if(currentState == DPAD_STATE)
            telemetry.addData("State", "Turning");
        else
            telemetry.addData("State", "???");
    }
}