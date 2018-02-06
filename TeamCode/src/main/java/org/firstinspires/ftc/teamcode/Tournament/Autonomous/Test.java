package org.firstinspires.ftc.teamcode.Tournament.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Tournament.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Tournament.HardwareMap.Zoinkifier;
@Disabled
@Autonomous(name = "Keenan I'm dead inside", group = "Autonomous")
public class Test extends LinearOpMode {

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


        //Flips the cube up
        robot.flipper.setTargetPosition(500);
        telemetry.addData("Blue", robot.flipper.getCurrentPosition());
        telemetry.update();
        robot.flipper.setPower(robot.FLIPPER_POWER * 0.75);
        while (robot.flipper.isBusy() && opModeIsActive())
        {       idle();
        telemetry.addData("Blue", robot.flipper.getCurrentPosition());
        telemetry.update();
    }
        robot.flipper.setPower(0);


        //Brings the flipper back down
        robot.flipper.setTargetPosition(0);
        robot.flipper.setPower(-robot.FLIPPER_POWER * 1 / 2);
        while (robot.flipper.isBusy() && opModeIsActive())
            idle();
        robot.flipper.setPower(0);
    }
}
