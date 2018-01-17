package org.firstinspires.ftc.teamcode.TalonCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TalonCode.HardwareMap.Zoinkifier;

@Autonomous(name = "Far Blue Auto", group = "Autonomous")
public class Far_Blue extends LinearOpMode {

    Zoinkifier robot;

    @Override
    public void runOpMode() {
        //Initialization
        robot = new Zoinkifier(hardwareMap, telemetry);
        robot.fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        //Drives until the robot is off the balancing stone, then stops
        robot.drive(.2,.2,.2,.2);

        do robot.updateGyro();
        while(Math.abs(robot.xRotation) < Math.toRadians(3) && Math.abs(robot.yRotation) < Math.toRadians(3));

        do robot.updateGyro();
        while(Math.abs(robot.xRotation) > Math.toRadians(3) && Math.abs(robot.yRotation) > Math.toRadians(3));

        robot.brake();
        //Maybe back up here?

        //Drives forward using the encoders
        robot.setDriveEncoders(.4,.4,.4,.4, 2000,2000,2000,2000);
        sleep(5000);

        robot.setDriveEncoders(.4,-.4, -.4, .4, 1000, - 1000, - 1000, 1000);
        sleep(5000);

        //Turns the robot until it is perpendicular to the cryptobox
        robot.updateGyro();
        while(robot.heading - Math.PI/2 > Math.toRadians(1)) {
            robot.drive(.1,.1,-.1,-.1);
            robot.updateGyro();
        }
        while(robot.heading - Math.PI/2 < Math.toRadians(-1)) {
            robot.drive(-.1,-.1,.1,.1);
            robot.updateGyro();
        }

        //Flips the block in
        robot.flipper.setTargetPosition(480);
        robot.flipper.setPower(-0.3);
        sleep(1000);

        //Puts the flipper back in its starting position
        robot.flipper.setTargetPosition(0);
        robot.flipper.setPower(0.3);
        sleep(1000);
    }
}
