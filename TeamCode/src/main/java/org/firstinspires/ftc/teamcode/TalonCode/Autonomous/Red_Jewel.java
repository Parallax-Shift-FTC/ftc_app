package org.firstinspires.ftc.teamcode.TalonCode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TalonCode.HardwareMap.Daquan;

@Autonomous(name = "Red Jewel Only", group = "Daquan")
public class Red_Jewel extends LinearOpMode {

    Daquan robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //Sets up the robot and waits for the start button to be pressed
        robot = new Daquan(hardwareMap, telemetry, false);
        waitForStart();

        //Moves the servo arm down
        robot.jewelArm.setPosition(1);

        wait(1000);

        robot.updateColor();

        //Change this depending on which way the sensor is facing
        if(robot.red < robot.blue * 3/2)
            robot.drive(robot.minPower, -robot.minPower, robot.minPower, -robot.minPower);
        else
            robot.drive(- robot.minPower, robot.minPower, -robot.minPower, robot.minPower);

        wait(500);
        robot.brake();
    }
}
