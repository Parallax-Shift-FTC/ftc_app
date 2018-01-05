package org.firstinspires.ftc.teamcode.TalonCode.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Zoinkifier {

    //Empty variables for the hardware map:
    //Fleft-bright are our mecanum drive motors, lintake/rintake are the Rev motors that spin our intake spinners, flipper is the motor on the flipper, and relicWinch will be the motor that powers our relic mechanism
    public DcMotor fleft, fright, bleft, bright, lintake, rintake, flipper;
    //Servo names are self-explanatory, the grabbers are on our relic mechanism
    public Servo topColorArm, baseColorArm, leftIntakeArm, rightIntakeArm;
    //Sensor names are self-explanatory
    public BNO055IMU gyroSensor;
    public ColorSensor colorSensor;

    //Public variables for other programs to utilize
    public double heading;
    public double red;
    public double blue;

    //Variables for our drive power; current is for individual instances to modify, the rest are constants
    public double currentDrivePower = .5;
    public static final double MIN_DRIVE_POWER = .1;
    public static final double MAX_DRIVE_POWER = 1;
    public static final double DRIVE_POWER = .5;
    //Other helpful constants, these have not yet been tested
    public static final double INTAKE_DEPLOYED_POSITION = .8;
    public static final double INTAKE_RETRACTED_POSITION = .2;
    public static final double INTAKE_POWER = .5;
    public static final double FLIPPER_POWER = .2;

    //Hardware map and telemetry variables allow for more interaction between this class and the one using it
    private HardwareMap hwMap;
    private Telemetry telemetry;

    //Constructor; Put in the hardware map and telemetry of your current program
    //Call this during initialization, add Vuforia later
    public Zoinkifier(HardwareMap hwmap, Telemetry telem) {
        hwMap = hwmap;
        telemetry = telem;

        telemetry.addData("Ready to begin", false);
        telemetry.update();

        //Setting up our motors
        fleft = hwMap.dcMotor.get("fleft");
        fright = hwMap.dcMotor.get("fright");
        bleft = hwMap.dcMotor.get("bleft");
        bright = hwMap.dcMotor.get("bright");
        lintake = hwMap.dcMotor.get("lintake");
        rintake = hwMap.dcMotor.get("rintake");
        flipper = hwMap.dcMotor.get("flipper");

        fright.setDirection(DcMotor.Direction.REVERSE);
        bright.setDirection(DcMotor.Direction.REVERSE);
        rintake.setDirection(DcMotor.Direction.REVERSE);

        //Setting up our servos and setting them to their initial positions (will have to modify)
        topColorArm = hwMap.servo.get("tarm");
        baseColorArm = hwMap.servo.get("barm");
        leftIntakeArm= hwMap.servo.get("left intake arm");
        rightIntakeArm = hwMap.servo.get("right intake arm");

        //Setting up sensors
        colorSensor = hwMap.colorSensor.get("color");
        gyroSensor = hwMap.get(BNO055IMU.class, "gyro");

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

    //A shortcut to move the intake servos to the active position
    public void deployIntake() {
        leftIntakeArm.setPosition(INTAKE_DEPLOYED_POSITION);
        rightIntakeArm.setPosition(INTAKE_DEPLOYED_POSITION);
    }

    //A shortcut to pull the intake servos back in
    public void retractIntake() {
        leftIntakeArm.setPosition(INTAKE_RETRACTED_POSITION);
        rightIntakeArm.setPosition(INTAKE_RETRACTED_POSITION);
    }

    //A shortcut for running the intake motors, put true to make it run and false to make it brake
    public void runIntake(double power) {
            lintake.setPower(power);
            rintake.setPower(power);
    }

    //Updates the heading variable; add pi/2 to make the starting angle 90 degrees instead of 0
    public void updateGyro() {
        heading = gyroSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle + Math.PI / 2;
        if(heading > Math.PI)
            heading = heading - 2 * Math.PI;
    }

    //Updates the color sensor variables
    public void updateColor() {
        red = colorSensor.red();
        blue = colorSensor.blue();
    }
}
