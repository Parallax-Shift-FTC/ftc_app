package org.firstinspires.ftc.teamcode.Tournament.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Tournament.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;

@Autonomous(name = "Close Red Auto", group = "Autonomous")
public class Close_Red extends LinearOpMode {

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

        //Sets up vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aa07QPX/////AAAAGT4IBGftwkAmodz5uX1NKehqWSuZYAizMXyJgDjbMQz+h5mPdKPRRA9id11R2ad9e3w3E6aS1Nep0aXgwwqRtAAmh6tizyQQZRM5qF+foaOh9zbuyAis/ANMODT0X5fAo3J6DqPNlOT9Es04EMKR5rIGhrb91rn3X+ferq2phtQ/PhQGHt44rkhNXSI1OV2GaY4BErnIgSktLZB6bWf49Jd3RtnybC9BfsuOv/2re0pEiGAiF+GyTV5pvuyVVFXFMKaiIR+aDe8qBpKV5z+ZUIWUC+z989ERqh9SKWdfJkOJt6glYFx/fEy3o4g8HwYfVbU+xU1fxufN+M3A2uZZaSSowVbbDDgr9CGxSd6/Dskg";
        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relictrackable = relicTrackables.get(0);
        relictrackable.setName("relicVuMark");
        relicTrackables.activate();

        telemetry.addData("Ready to begin", true);
        telemetry.update();
        waitForStart();

        //Puts the intake in its starting position
        robot.deployIntake();

        //Waits until the robot scans the vumark, then figures out what cryptobox to put the glyph
        //in anc closes vuforia to conserve battery. If vuforia doesn't work after 5 seconds, it
        //times out and just goes for the closest slot
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relictrackable);
        robot.timer.reset();
        while(vuMark == RelicRecoveryVuMark.UNKNOWN && robot.timer.seconds() < 10 && opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relictrackable);
            idle();
        }
        if(vuMark == RelicRecoveryVuMark.UNKNOWN)
            strafeDistance = robot.CLOSE_STONE_CLOSE_SLOT;
        if(vuMark == RelicRecoveryVuMark.RIGHT)
            strafeDistance = robot.CLOSE_STONE_CLOSE_SLOT;
        else if(vuMark == RelicRecoveryVuMark.CENTER)
            strafeDistance = robot.CLOSE_STONE_MIDDLE_SLOT;
        else
            strafeDistance = robot.CLOSE_STONE_FAR_SLOT;
        vuforia.close();

        //Moves the sensor into position and takes a reading from the color sensor, then hits the
        //correct colored jewel and folds the jewel arm back up
        robot.bottomServo.setPosition(0.55);
        robot.topServo.setPosition(0.75);
        sleep(200);
        robot.bottomServo.setPosition(0.68);
        sleep(500);
        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() > robot.colorSensor.blue())
            robot.topServo.setPosition(.86);
        else if(robot.colorSensor.red() < robot.colorSensor.blue())
            robot.topServo.setPosition(.60);
        sleep(1000);
        robot.bottomServo.setPosition(0.55);
        sleep(200);
        robot.topServo.setPosition(0.25);
        robot.bottomServo.setPosition(0.03);
        sleep(500);

        //Starts driving, then uses the rotation about the y axis to tell when the robot has gotten
        //off the balancing stone and stops
        robot.drive(-.2,-.2,-.2,-.2);
        telemetry.addData("State", "Drive until Tilted");
        telemetry.update();
        do {
            robot.updateGyro();
            idle();
        }
        while(robot.yRotation < Math.toRadians(2) && opModeIsActive());
        telemetry.addData("State", "Drive until not Tilted");
        telemetry.update();
        sleep(250);
        do {
            robot.updateGyro();
            idle();
        }
        while(robot.yRotation > Math.toRadians(2) && opModeIsActive());
        robot.brake();

        //Drives forward using the encoders
        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        robot.setDriveEncoders(-.4,-.4,-.4,-.4, -400,-400,-400,-400);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        //Turn 90 degrees to line up with cryptobox
        telemetry.addData("State", "Turn 90");
        telemetry.update();
        robot.updateGyro();
        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.updateGyro();
        double error = robot.heading;
        while(error > Math.toRadians(6) && opModeIsActive()) {
            robot.updateGyro();
            error = robot.heading;
            robot.drive( .1 + error * 0.4 / (Math.PI/2), - .1 - error * 0.4 / (Math.PI/2),  .1 + error * 0.4 / (Math.PI/2), - .1 - error * 0.4 / (Math.PI/2));
            idle();
        }
        robot.brake();

        //Drives forward using the encoders
        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        robot.setDriveEncoders(-.4,-.4,-.4,-.4, -900,-900,-900,-900);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        //Strafes sideways the correct amount to line up with the correct slot
        telemetry.addData("State", "Drive Sideways With Encoders");
        telemetry.update();
        //Drives to the correct place using the encoders
        robot.setDriveEncoders(.2, -.2, -.2, .2, strafeDistance, -strafeDistance, -strafeDistance, strafeDistance);
        while (robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        //Flips the cube up
        robot.flipper.setTargetPosition(500);
        robot.flipper.setPower(robot.FLIPPER_POWER * 0.75);
        while(robot.flipper.isBusy() && opModeIsActive())
            idle();
        robot.flipper.setPower(0);

        //Drives back, goes forward again to hit the cube in, then drives back again to park
        robot.setDriveEncoders(.2,.2,.2,.2, 500, 500, 500, 500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.setDriveEncoders(-.2,-.2,-.2,-.2, -600, -600, -600, -600);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.setDriveEncoders(.2,.2,.2,.2, 500, 500, 500, 500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        //Brings the flipper back down
        robot.flipper.setTargetPosition(0);
        robot.flipper.setPower(-robot.FLIPPER_POWER * 1/2);
        while(robot.flipper.isBusy() && opModeIsActive())
            idle();
        robot.flipper.setPower(0);
    }
}
