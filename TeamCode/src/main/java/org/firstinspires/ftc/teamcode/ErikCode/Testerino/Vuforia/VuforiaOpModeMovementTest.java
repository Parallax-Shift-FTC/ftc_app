package org.firstinspires.ftc.teamcode.ErikCode.Testerino.Vuforia;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="VuMark Movement Test", group ="Vumark")
public class VuforiaOpModeMovementTest extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    DcMotor fleft;
    DcMotor fright;
    DcMotor bleft;
    DcMotor bright;

    @Override
    public void runOpMode()
    {
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");
        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("f right");

        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aa07QPX/////AAAAGT4IBGftwkAmodz5uX1NKehqWSuZYAizMXyJgDjbMQz+h5mPdKPRRA9id11R2ad9e3w3E6aS1Nep0aXgwwqRtAAmh6tizyQQZRM5qF+foaOh9zbuyAis/ANMODT0X5fAo3J6DqPNlOT9Es04EMKR5rIGhrb91rn3X+ferq2phtQ/PhQGHt44rkhNXSI1OV2GaY4BErnIgSktLZB6bWf49Jd3RtnybC9BfsuOv/2re0pEiGAiF+GyTV5pvuyVVFXFMKaiIR+aDe8qBpKV5z+ZUIWUC+z989ERqh9SKWdfJkOJt6glYFx/fEy3o4g8HwYfVbU+xU1fxufN+M3A2uZZaSSowVbbDDgr9CGxSd6/Dskg";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relictrackable = relicTrackables.get(0);
        relictrackable.setName("relicVuMark");

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive())
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relictrackable);
            if(vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                telemetry.addData("Vumark",vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relictrackable.getListener()).getPose();
                if(pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

                    double X = trans.get(0);
                    double Y = trans.get(1);
                    double Z = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                }
                if(vuMark == RelicRecoveryVuMark.LEFT)
                {
                    bright.setPower(.2);
                    fright.setPower(.2);
                    bleft.setPower(.2);
                    fleft.setPower(.2);
                    sleep(100);
                    bright.setPower(0);
                    fright.setPower(0);
                    bleft.setPower(0);
                    fleft.setPower(0);
                }
                else if(vuMark == RelicRecoveryVuMark.CENTER)
                {
                    bright.setPower(-.2);
                    fright.setPower(-.2);
                    bleft.setPower(-.2);
                    fleft.setPower(-.2);
                    sleep(100);
                    bright.setPower(0);
                    fright.setPower(0);
                    bleft.setPower(0);
                    fleft.setPower(0);
                }
                else if(vuMark == RelicRecoveryVuMark.RIGHT)
                {
                    bright.setPower(-.2);
                    fright.setPower(-.2);
                    bleft.setPower(.2);
                    fleft.setPower(.2);
                    sleep(100);
                    bright.setPower(0);
                    fright.setPower(0);
                    bleft.setPower(0);
                    fleft.setPower(0);
                }
                else
                {
                    bright.setPower(0);
                    fright.setPower(0);
                    bleft.setPower(0);
                    fleft.setPower(0);
                }
            }
            else
                telemetry.addData("Vumark","Not Visible");
            telemetry.update();
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


}
