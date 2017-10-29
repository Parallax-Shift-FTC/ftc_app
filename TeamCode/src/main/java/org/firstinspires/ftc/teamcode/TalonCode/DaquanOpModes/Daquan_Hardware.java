package org.firstinspires.ftc.teamcode.TalonCode.DaquanOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
-Name: Daquan Hardware
- Creator[s]: Talon
-Date Created: 10/28/2017
-Objective: Create a class that has basic functions and variables, and also sets up the hardware map
 for our test robot named Daquan
 */

public class Daquan_Hardware {

    //Empty variables for the hardware map
    public DcMotor fleft, fright, bleft, bright /*lurricane, rurricane*/; //Uncomment the hurricanes out once we get the second REV module
    public BNO055IMU gyro;

    //Public variables for other programs to utilize
    public double heading;
    public static final double DRIVE_POWER = 0.3;
    //public static final float INTAKE_POWER = 0.16f;

    //Hardware map and telemetry variables allow for more interaction between this class and the one using it
    private HardwareMap hwMap;
    private Telemetry telemetry;

    //Constructor; Put in the hardware map and telemetry of your current program, and true or false depending on if you will be using the gyro sensor
    //Call this during initialization
    public Daquan_Hardware(HardwareMap hwmap, Telemetry telem, boolean usesGyro) {
        hwMap = hwmap;
        telemetry = telem;

        telemetry.addData("Ready to begin", false);
        telemetry.update();

        fleft = hwMap.dcMotor.get("fleft");
        fright = hwMap.dcMotor.get("fright");
        bleft = hwMap.dcMotor.get("bleft");
        bright = hwMap.dcMotor.get("bright");

        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);

        //rurricane.setDirection(DcMotor.Direction.REVERSE);

        //Sets the motors to run based on speed and not power
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets up the gyro sensor if necessary
        if(usesGyro) {
            gyro = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro.initialize(parameters);
        }

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

    //A shortcut for turning the robot; for direction clockwise is -1 and counter-clockwise is 1
    public void turn(double direction, double power) {
        fleft.setPower(- direction * power);
        fright.setPower(direction * power);
        bleft.setPower(- direction * power);
        bright.setPower(direction * power);
    }

    //Updates the heading variable; Normally add Pi/2 but this subtracts Pi/2 because the gyro sensor is backwards
    public void updateGyro() {
        heading = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle - Math.PI / 2;
    }
}
