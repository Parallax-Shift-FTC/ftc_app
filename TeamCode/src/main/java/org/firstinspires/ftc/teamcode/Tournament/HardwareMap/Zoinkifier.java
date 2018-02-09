package org.firstinspires.ftc.teamcode.Tournament.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Zoinkifier {

    //Empty variables for the hardware map:
    //Fleft, fright, bleft, bright(HD Hex REV Motor) are our mecanum drive motors, lintake/rintake
    //(Core Hex REV Motor) turn our intake spinners, and flipper(Andymark NeveRest 40) is the motor
    //attached to the flipper
    public DcMotor fleft, fright, bleft, bright, lintake, rintake, flipper;
    //leftIntakeArm and rightIntakeArm are the servos that the intake is mounted on, and topServo
    //and bottomServo are the servos on the jewel hitter
    public Servo leftIntakeArm, rightIntakeArm, topServo, bottomServo;
    //gyroSensor is a BNO055IMU sensor inside our REV Hub, colorSensor is a REV Color/Range Sensor
    public BNO055IMU gyroSensor;
    public ColorSensor colorSensor;

    //Gyro Sensor Variables
    public double heading;
    public double yRotation;

    //Variables for our drive power in tele-op; currentDrivePower is for individual instances to
    //modify, DRIVE_POWER is the main cruising speed, and SLOW_POWER is for aligning with the
    //cryptobox
    public static final double FLIPPER_POWER = .45;
    public static final double INTAKE_POWER = -1;

    public static final double SLOW_POWER = 0.25;
    public static final double DRIVE_POWER = .85;
    public double currentDrivePower = DRIVE_POWER;
    //Powers for the flipper and intake motors
    //Encoder values for lining up with the cryptobox in autonomous
    public static final int FAR_STONE_CLOSE_SLOT = 300;
    public static final int FAR_STONE_MIDDLE_SLOT = 1050;
    public static final int FAR_STONE_FAR_SLOT = 1700;
    public static final int CLOSE_STONE_CLOSE_SLOT = 300;
    public static final int CLOSE_STONE_MIDDLE_SLOT = 1050;
    public static final int CLOSE_STONE_FAR_SLOT = 1700;

    //Sets up a timer for other programs to use
    public static ElapsedTime timer = new ElapsedTime();

    //Empty hardwareMap variable to be filled in the constructor
    private HardwareMap hardwareMap;

    //Constructor; Put in the hardware map and telemetry of your current program, call during init()
    public Zoinkifier(HardwareMap harwareMap) {
        //This allows us to access the hardware of the program that uses the constructor
        hardwareMap = harwareMap;

        //Setting up our motors
        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("fright");
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");
        lintake = hardwareMap.dcMotor.get("lintake");
        rintake = hardwareMap.dcMotor.get("rintake");
        flipper = hardwareMap.dcMotor.get("flipper");

        //Reversing motors so that the drive motors and intake motors go the same way when given a
        //positive value
        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);

        //Setting up our servos
        leftIntakeArm= hardwareMap.servo.get("left intake arm");
        rightIntakeArm = hardwareMap.servo.get("right intake arm");
        bottomServo = hardwareMap.servo.get("bottom servo");
        topServo = hardwareMap.servo.get("top servo");

        //Reversing the right intake servo so that the intake servos go the same way
        rightIntakeArm.setDirection(Servo.Direction.REVERSE);

        //Setting up sensors
        colorSensor = hardwareMap.colorSensor.get("color sensor");
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

    //A shortcut to set the intake servos to their normal position
    public void deployIntake() {
        leftIntakeArm.setPosition(.755);
        rightIntakeArm.setPosition(.845);
    }

    //A shortcut to move the intake servos all the way out
    public void farOut() {
        leftIntakeArm.setPosition(.9);
        rightIntakeArm.setPosition(.9);
    }

    //A shortcut for running the intake motors, put intake wheel powers in for parameters
    public void runIntake(double leftpower, double rightpower) {
        lintake.setPower(leftpower);
        rintake.setPower(rightpower);
    }

    //A shortcut to set the encoder values for the robot and set the power to them
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

    //Updates the heading variable; add pi/2 to make the starting angle 90 degrees instead of 0 for
    //the heading variable, yRotation is used for autonomous
    public void updateGyro() {
        heading = gyroSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle + Math.PI / 2;
        if(heading > Math.PI)
            heading = heading - 2 * Math.PI;
        yRotation = gyroSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).secondAngle;
    }
}
