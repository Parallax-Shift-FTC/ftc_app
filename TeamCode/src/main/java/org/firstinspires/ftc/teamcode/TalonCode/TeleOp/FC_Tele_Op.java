package org.firstinspires.ftc.teamcode.TalonCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TalonCode.HardwareMap.Zoinkifier;


@TeleOp(name = "Talon's Main TeleOp", group = "Zoinkifier")
public class FC_Tele_Op extends OpMode {

    Zoinkifier robot;

    //Sets up states for the state machine
    int currentState = 0;
    static final int FIELD_CENTRIC_STATE = 0;
    static final int ROBOT_CENTRIC_STATE = 1;

    //Target angle for turning
    double targetAngle;
    double turningSpeed;

    //The angle that the robot should drive at relative to its starting position, Pi/2 corresponds to straight forward
    double angleFromDriver = Math.PI / 2;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        robot = new Zoinkifier(hardwareMap, telemetry);
    }

    @Override
    public void loop (){

        robot.updateGyro();
        telemetry.addData("Gyro Heading", Math.toDegrees(robot.heading));

        //Ultra turbo and sneak modes
        if(gamepad1.left_bumper)
            robot.currentDrivePower = robot.MAX_DRIVE_POWER;
        else if(gamepad1.dpad_down)
            robot.currentDrivePower = robot.MIN_DRIVE_POWER;
        else
            robot.currentDrivePower = robot.DRIVE_POWER;

        //Changes the angle of the robot's motion with respect to the driver to its current angle when the start button is pressed on gamepad one
        if(gamepad1.back)
            angleFromDriver = robot.heading;

        //Deploys or retracts the intake spinners if the up or down button is pressed on the dpad on the second controller
        if(gamepad2.dpad_right)
            robot.deployIntake();
        else if(gamepad2.dpad_left)
            robot.retractIntake();

        //Runs the intake spinners when a is pressed on the second gamepad
        if(gamepad2.a)
            robot.runIntake(true);
        else
            robot.runIntake(false);

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
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.left_stick_x) * robot.currentDrivePower,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.left_stick_x) * robot.currentDrivePower,
                        (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (gamepad1.left_stick_x) * robot.currentDrivePower,
                        (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower * robot.currentDrivePower + (- gamepad1.left_stick_x) * robot.currentDrivePower
                );

                //If the right bumper is held it will go to the robot centric state
                if(gamepad1.start)
                    currentState = ROBOT_CENTRIC_STATE;

                break;

            case ROBOT_CENTRIC_STATE:
                //Arcade drive with right joystick, turn with triggers(clockwise-right, counterclockwise-left)
                //Format: +/- Turning +/- Forward/Backward +/- Strafing
                robot.drive(
                        robot.currentDrivePower * (gamepad1.left_stick_x) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x),
                        robot.currentDrivePower * (- gamepad1.left_stick_x) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x),
                        robot.currentDrivePower * (gamepad1.left_stick_x) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (- gamepad1.right_stick_x),
                        robot.currentDrivePower * (- gamepad1.left_stick_x) + robot.currentDrivePower * (- gamepad1.right_stick_y) + robot.currentDrivePower * (gamepad1.right_stick_x)
                );


                if(gamepad1.b)
                    currentState = FIELD_CENTRIC_STATE;

                break;
        }

        //Tells the driver the current state
        if(currentState == FIELD_CENTRIC_STATE)
            telemetry.addData("State", "Field Centric");
        else if(currentState == ROBOT_CENTRIC_STATE)
            telemetry.addData("State", "Robot Centric");
        else
            telemetry.addData("State", "???");
    }
}