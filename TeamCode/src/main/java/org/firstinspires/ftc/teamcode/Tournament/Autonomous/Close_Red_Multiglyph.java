package org.firstinspires.ftc.teamcode.Tournament.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Tournament.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;

@Autonomous(name = "get cube?", group = "Autonomous")
public class Close_Red_Multiglyph extends LinearOpMode {

    Zoinkifier robot;
    ClosableVuforiaLocalizer vuforia;
    int strafeDistance;

    @Override
    public void runOpMode() {
        //Initialization
        telemetry.addData("Ready to begin", false);
        telemetry.update();

        robot = new Zoinkifier(hardwareMap);

        //Resets the encoders
        robot.fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the correct modes for the motors
        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initializes the jewel arm servos
        robot.bottomServo.setPosition(0);
        robot.topServo.setPosition(0.25);

        waitForStart();

        robot.lintake.setPower(robot.INTAKE_POWER);
        robot.rintake.setPower(robot.INTAKE_POWER);
        robot.deployIntake();
        robot.setDriveEncoders(.6,.6,.6,.6, 3500, 3500, 3500, 3500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        sleep(1000);

        robot.setDriveEncoders(-.6,-.6,-.6,-.6, -1000, -1000, -1000, -1000);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        robot.lintake.setPower(0);
        robot.rintake.setPower(0);

        robot.farOut();

        robot.setDriveEncoders(-.6,.6,-.6,.6, -1000, 1000, -1000, 1000);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        robot.lintake.setPower(robot.INTAKE_POWER);
        robot.rintake.setPower(robot.INTAKE_POWER);
        robot.deployIntake();
        sleep(1000);

        robot.lintake.setPower(-0.2);
        robot.rintake.setPower(0.2);
        sleep(250);

        robot.lintake.setPower(robot.INTAKE_POWER);
        robot.rintake.setPower(robot.INTAKE_POWER);
        sleep(500);

        robot.lintake.setPower(-0.2);
        robot.rintake.setPower(0.2);
        sleep(300);

        robot.lintake.setPower(robot.INTAKE_POWER);
        robot.rintake.setPower(robot.INTAKE_POWER);
        sleep(500);

        robot.lintake.setPower(0);
        robot.rintake.setPower(0);

        robot.setDriveEncoders(.6,-.6,.6,-.6, 1000, -1000, 1000, -1000);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        robot.setDriveEncoders(-.6,-.6,-.6,-.6, -2500, -2500, -2500, -2500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();
    }
}
