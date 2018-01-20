package org.firstinspires.ftc.teamcode.Tournament.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Tournament.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;

@Autonomous(name = "Close Auto", group = "Autonomous")
public class Close_Red extends LinearOpMode {

    Zoinkifier robot;
    ClosableVuforiaLocalizer vuforia;
    int strafeDistance;

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

        robot.bottomServo.setPosition(0.03);
        robot.topServo.setPosition(0.25);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aa07QPX/////AAAAGT4IBGftwkAmodz5uX1NKehqWSuZYAizMXyJgDjbMQz+h5mPdKPRRA9id11R2ad9e3w3E6aS1Nep0aXgwwqRtAAmh6tizyQQZRM5qF+foaOh9zbuyAis/ANMODT0X5fAo3J6DqPNlOT9Es04EMKR5rIGhrb91rn3X+ferq2phtQ/PhQGHt44rkhNXSI1OV2GaY4BErnIgSktLZB6bWf49Jd3RtnybC9BfsuOv/2re0pEiGAiF+GyTV5pvuyVVFXFMKaiIR+aDe8qBpKV5z+ZUIWUC+z989ERqh9SKWdfJkOJt6glYFx/fEy3o4g8HwYfVbU+xU1fxufN+M3A2uZZaSSowVbbDDgr9CGxSd6/Dskg";
        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relictrackable = relicTrackables.get(0);
        relictrackable.setName("relicVuMark");

        relicTrackables.activate();

        waitForStart();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relictrackable);
        while(vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relictrackable);
            idle();
        }

        if(vuMark == RelicRecoveryVuMark.RIGHT)
            strafeDistance = robot.CLOSE_STONE_CLOSE_SLOT;
        else if(vuMark == RelicRecoveryVuMark.CENTER)
            strafeDistance = robot.CLOSE_STONE_MIDDLE_SLOT;
        else
            strafeDistance = robot.CLOSE_STONE_FAR_SLOT;
        vuforia.close();
        robot.deployIntake();

        robot.bottomServo.setPosition(0.55);
        robot.topServo.setPosition(0.72);
        sleep(200);
        robot.bottomServo.setPosition(0.68);
        sleep(500);

        //Hits appropriate jewel
        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() > robot.colorSensor.blue())
            robot.topServo.setPosition(.60);
        else if(robot.colorSensor.red() < robot.colorSensor.blue())
            robot.topServo.setPosition(.86);

        sleep(1000);
        robot.bottomServo.setPosition(0.55);
        sleep(200);
        robot.topServo.setPosition(0.25);
        robot.bottomServo.setPosition(0.03);
        sleep(500);

        //Drives until the robot is off the balancing stone, then stops
        robot.drive(-.2,-.2,-.2,-.2);

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
        robot.setDriveEncoders(-.4,-.4,-.4,-.4, -500,-500,-500,-500);

        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        robot.brake();

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

        telemetry.addData("State", "Drive Forward With Encoders");
        telemetry.update();
        //Drives forward using the encoders
        robot.setDriveEncoders(-.4,-.4,-.4,-.4, -200,-200,-200,-200);

        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        telemetry.addData("State", "Drive Sideways With Encoders");
        telemetry.update();
        //Drives to the correct place using the encoders
        robot.setDriveEncoders(-.2, .2, .2, -.2, -strafeDistance, strafeDistance, strafeDistance, -strafeDistance);

        while (robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();
        robot.brake();

        robot.flipper.setTargetPosition(500);
        robot.flipper.setPower(robot.FLIPPER_POWER * 0.75);
        while(robot.flipper.isBusy() && opModeIsActive())
            idle();

        robot.flipper.setPower(0);

        sleep(1000);

        robot.setDriveEncoders(.2,.2,.2,.2, 500, 500, 500, 500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        robot.setDriveEncoders(-.2,-.2,-.2,-.2, -600, -600, -600, -600);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        robot.setDriveEncoders(.2,.2,.2,.2, 500, 500, 500, 500);
        while(robot.fleft.isBusy() && robot.fright.isBusy() && opModeIsActive())
            idle();

        robot.flipper.setTargetPosition(0);
        robot.flipper.setPower(-robot.FLIPPER_POWER * 1/2);
        while(robot.flipper.isBusy() && opModeIsActive())
            idle();

        robot.flipper.setPower(0);
    }
}
