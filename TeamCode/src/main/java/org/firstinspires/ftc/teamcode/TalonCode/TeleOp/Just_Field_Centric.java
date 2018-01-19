package org.firstinspires.ftc.teamcode.TalonCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TalonCode.HardwareMap.Zoinkifier;

@Disabled
@TeleOp(name = "Talon's Simple Field Centric", group = "Zoinkifier")
public class Just_Field_Centric extends OpMode {

    Zoinkifier robot;

    //The angle that the robot should drive at relative to its starting position, Pi/2 corresponds to straight forward
    static final double ANGLE_FROM_DRIVER = Math.PI / 2;

    @Override   //Sets up the robot class so we can use its hardware map and variables
    public void init (){
        //robot = new Daquan(hardwareMap, telemetry, true);
    }

    @Override
    public void loop (){

        //Ultra turbo and sneak modes
        if(gamepad1.left_bumper)
            robot.currentDrivePower = robot.MAX_DRIVE_POWER;
        else if(gamepad1.dpad_down)
            robot.currentDrivePower = robot.MIN_DRIVE_POWER;
        else
            robot.currentDrivePower = robot.DRIVE_POWER;

        robot.updateGyro();

        //This is the angle that the right joystick is pointing in
        double inputAngle = Math.atan2(- gamepad1.right_stick_y, gamepad1.right_stick_x);

        //This is the magnitude of how far the joystick is pushed
        double inputPower = Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);

        if(inputPower > 1)
            inputPower = 1;

        //This is the angle at which the robot should translate
        double moveAngle = inputAngle + (ANGLE_FROM_DRIVER - robot.heading);

        robot.drive (
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower / 2 + gamepad1.right_trigger - gamepad1.left_trigger,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower / 2 - gamepad1.right_trigger + gamepad1.left_trigger,
                (Math.sin(moveAngle) - Math.cos(moveAngle)) * inputPower / 2 + gamepad1.right_trigger - gamepad1.left_trigger,
                (Math.sin(moveAngle) + Math.cos(moveAngle)) * inputPower / 2 - gamepad1.right_trigger + gamepad1.left_trigger
        );
        telemetry.addData("Gyro Heading", robot.heading);
        telemetry.addData("Joystick Direction", Math.toDegrees(inputAngle));
        telemetry.addData("Joystick Magnitude", inputPower);
    }
}