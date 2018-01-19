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

        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        //Drives until the robot is off the balancing stone, then stops
        robot.drive(.2,.2,.2,.2);

        telemetry.addData("State", "Drive until Tilted");
        telemetry.update();

        do {
            robot.updateGyro();
            idle();
        }
        while(robot.yRotation < Math.toRadians(1) && opModeIsActive());

        telemetry.addData("State", "Drive until not Tilted");
        telemetry.update();

        sleep(250);

        do {
            robot.updateGyro();
            idle();
        }
        while(robot.yRotation > Math.toRadians(1) && opModeIsActive());

        robot.brake();

        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        //Drives forward using the encoders
        robot.setDriveEncoders(.4,.4,.4,.4, 500,500,500,500);

        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        robot.brake();

        telemetry.addData("State", "Do a 180");
        telemetry.update();
        robot.updateGyro();

        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updateGyro();
        double error = robot.heading + Math.PI / 2;
        while(error > Math.toRadians(6) && opModeIsActive()) {
            robot.updateGyro();
            error = robot.heading + Math.PI / 2;
            robot.drive( .1 + error * 0.4 / Math.PI, - .1 - error * 0.4 / Math.PI,  .1 + error * 0.4 / Math.PI, - .1 - error * 0.4 / Math.PI);
            idle();
        }
        robot.brake();

/*        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        //Drives forward using the encoders
        robot.setDriveEncoders(-.4,-.4,-.4,-.4, -500,-500,-500,-500);

        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();*/
        /*
        telemetry.addData("State", "Drive Sideways With Encoders");
        telemetry.update();
        robot.setDriveEncoders(.4,-.4, -.4, .4, 1000, - 1000, - 1000, 1000);
        sleep(5000);

        telemetry.addData("State", "Turn to Correct Angle");
        telemetry.update();
        //Turns the robot until it is perpendicular to the cryptobox
        robot.updateGyro();
        while(robot.heading + Math.PI/2 > Math.toRadians(1)) {
            robot.drive(.1,.1,-.1,-.1);
            robot.updateGyro();
        }
        while(robot.heading + Math.PI/2 < Math.toRadians(-1)) {
            robot.drive(-.1,-.1,.1,.1);
            robot.updateGyro();
        }

        telemetry.addData("State", "Flip up");
        telemetry.update();
        //Flips the block in
        robot.flipper.setTargetPosition(480);
        robot.flipper.setPower(-0.3);
        sleep(1000);

        telemetry.addData("State", "Flip back down");
        telemetry.update();
        //Puts the flipper back in its starting position
        robot.flipper.setTargetPosition(0);
        robot.flipper.setPower(0.3);
        sleep(1000);*/
    }
}
