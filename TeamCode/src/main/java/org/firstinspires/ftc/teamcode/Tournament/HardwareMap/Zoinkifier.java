package org.firstinspires.ftc.teamcode.Tournament.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Zoinkifier {

    //Empty variables for the hardware map:
    //Fleft-bright are our mecanum drive motors, lintake/rintake are the Rev motors that spin our intake spinners, flipper is the motor on the flipper, and relicWinch will be the motor that powers our relic mechanism
    public DcMotor fleft, fright, bleft, bright, lintake, rintake, flipper;
    //Servo names are self-explanatory, the grabbers are on our relic mechanism
    public Servo leftIntakeArm, rightIntakeArm, bottomServo, topServo;
    //Sensor names are self-explanatory
    public BNO055IMU gyroSensor;
    
    public ColorSensor colorSensor;

    //Public variables for other programs to utilize
    public double heading;
    public double xRotation;
    public double yRotation;

    //Vuforia Variables
    public static final String TAG = "Vuforia VuMark Sample";

    //Variables for our drive power; current is for individual instances to modify, the rest are constants
    public double currentDrivePower = 1;
    public static final double MIN_DRIVE_POWER = .1;
    public static final double MAX_DRIVE_POWER = 1;
    public static final double DRIVE_POWER = 0.65;
    //Other helpful constants
    public static final double INTAKE_POWER = 1;
    public static final double FLIPPER_POWER = .5;

    public static final int FAR_STONE_CLOSE_SLOT = 450;
    public static final int FAR_STONE_MIDDLE_SLOT = 1250;
    public static final int FAR_STONE_FAR_SLOT = 2050;

    public static final int CLOSE_STONE_CLOSE_SLOT = 450;
    public static final int CLOSE_STONE_MIDDLE_SLOT = 1250;
    public static final int CLOSE_STONE_FAR_SLOT = 2050;

    //Hardware map and telemetry variables allow for more interaction between this class and the one using it
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    //Constructor; Put in the hardware map and telemetry of your current program
    //Call this during initialization, add Vuforia later
    public Zoinkifier(HardwareMap hardwareMap, Telemetry telem) {
        hardwareMap = hardwareMap;
        telemetry = telem;

        telemetry.addData("Ready to begin", false);
        telemetry.update();

        //Setting up our motors
        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("fright");
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");
        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        flipper = hardwareMap.dcMotor.get("flipper");

        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);
        lintake.setDirection(DcMotor.Direction.REVERSE);

        //Setting up our servos and setting them to their initial positions (will have to modify)
        leftIntakeArm= hardwareMap.servo.get("left intake arm");
        rightIntakeArm = hardwareMap.servo.get("right intake arm");
        bottomServo = hardwareMap.servo.get("bottom servo");
        topServo = hardwareMap.servo.get("top servo");

        bottomServo.setPosition(0.03);
        topServo.setPosition(0.25);

        rightIntakeArm.setDirection(Servo.Direction.REVERSE);
        
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        //Setting up sensors
        gyroSensor = hardwareMap.get(BNO055IMU.class, "gyro");

        //Additional gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyroSensor.initialize(parameters);

        telemetry.addData("Ready to begin", true);
        telemetry.update();
    }

    //A shortcut for moving the robot wheels
    public void drive(double fl, double fr, double bl, double br) {
        fleft.setPower(fl);
        fright.setPower(fr);
        bleft.setPower(bl);
        bright.setPower(br);
    }

    //A shortcut for braking
    public void brake() {
        fleft.setPower(0);
        fright.setPower(0);
        bleft.setPower(0);
        bright.setPower(0);
    }

    //A shortcut to set the intake servos to their usual position
    public void deployIntake() {
        leftIntakeArm.setPosition(.85);
        rightIntakeArm.setPosition(.90);
    }

    //A shortcut to move the intake servos all the way out
    public void farOut() {
        leftIntakeArm.setPosition(.95);
        rightIntakeArm.setPosition(1);
    }

    //A shortcut for running the intake motors, put intake wheel powers in for parameters
    public void runIntake(double leftpower, double rightpower) {
        lintake.setPower(leftpower);
        rintake.setPower(rightpower);
    }

    public void setDriveEncoders(double powerfl, double powerfr, double powerbl, double powerbr, int fl, int fr, int bl, int br) {
        fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fleft.setTargetPosition(fl + fleft.getCurrentPosition());
        fright.setTargetPosition(fr + fright.getCurrentPosition());
        bleft.setTargetPosition(bl + bleft.getCurrentPosition());
        bright.setTargetPosition(br + bright.getCurrentPosition());

        fleft.setPower(powerfl);
        fright.setPower(powerfr);
        bleft.setPower(powerbl);
        bright.setPower(powerbr);
    }

    //Updates the heading variable; add pi/2 to make the starting angle 90 degrees instead of 0
    public void updateGyro() {
        heading = gyroSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle + Math.PI / 2;
        if(heading > Math.PI)
            heading = heading - 2 * Math.PI;
        yRotation = gyroSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).secondAngle;
        xRotation = gyroSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).thirdAngle;
    }
}
