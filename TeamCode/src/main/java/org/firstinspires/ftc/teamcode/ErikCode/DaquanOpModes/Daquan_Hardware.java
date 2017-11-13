package org.firstinspires.ftc.teamcode.ErikCode.DaquanOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
- Name: Holonomic Hardware Map
- Creator[s]: Talon
- Date Created: 6/16/17
- Objective: To create a class that sets up the hardware map for our holonomic robot and has basic
             functions to reduce redundancies in other programs.
 */

public class Daquan_Hardware {

    //Declaring variables
    public ColorSensor colorSensor;
    public DcMotor fleft, fright, bleft, bright;
    public BNO055IMU gyro;
    public double heading;
    public double dp = 0.4f; //Drive Power (range = 0-1)
    private HardwareMap hwMap;
    private Telemetry telemetry;
    public ElapsedTime time = new ElapsedTime();
    double currentDrivePower;
    public double red;
    public double blue;
    public double green;
    public double brightness;

    //Constructor; Put program's hardwaremap first, then telemetry,  then put true if gyro will be used or false if it won't
    public Daquan_Hardware(HardwareMap hwmap, Telemetry telem, boolean usesGyro) {

        hwMap = hwmap;
        telemetry = telem;

        //Signifies that initialization is not yet finished
        telemetry.addData("Ready to go", false);
        telemetry.update();

        //Setting up drive motors
        fleft = hwMap.dcMotor.get("fleft");
        fright = hwMap.dcMotor.get("fright");
        bleft = hwMap.dcMotor.get("bleft");
        bright = hwMap.dcMotor.get("bright");
        fright.setDirection(DcMotor.Direction.REVERSE);
        bright.setDirection(DcMotor.Direction.REVERSE);

        colorSensor = hwMap.colorSensor.get("color");

        //Setting up gyro sensor if necessary
        if (usesGyro) {
            gyro = hwMap.get(BNO055IMU.class, "imu");
            //Setting up data for gyro sensors
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro.initialize(parameters); //this could also be causingissues but is probably important
        }

        //Alerts user that initialization is done
        telemetry.addData("Ready to go", true);
        telemetry.update();
    }

    public void drive(double fl, double fr, double bl, double br) {
        fleft.setPower(ClipValue(fl));
        fright.setPower(ClipValue(fr));
        bleft.setPower(ClipValue(bl));
        bright.setPower(ClipValue(br));
    }

    public void updateGyro() {
        //May not have to make negative? Make it so that turning is CCW
        heading = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        if (heading > 0) {
            heading = heading + 0;
        } else {
            heading = heading + 2 * Math.PI;
        }
    }

    double ClipValue(double value) {
        if (value > dp || value < -dp)
            return ((Math.abs(value) / value) * dp);
        else
            return value;
    }

    public void UpdateColor() {
        red = colorSensor.red();
        blue = colorSensor.blue();
        green = colorSensor.green();
        brightness = colorSensor.alpha();
    }
}
