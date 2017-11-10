package org.firstinspires.ftc.teamcode.TalonCode.DaquanOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Brown on 11/10/2017.
 */
@Autonomous(name = "Talon's Field Centric", group = "Daquan")
public class Red_Autonomous extends LinearOpMode {

    Daquan_Hardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //Sets up the robot and waits for the start button to be pressed
        robot = new Daquan_Hardware(hardwareMap, telemetry, false);
        waitForStart();

        //Moves the servo arm down
        robot.jewelArm.setPosition(1);

        wait(1000);

        robot.updateColor();

        if(robot.red > robot.blue * 3/2)
            robot.drive(robot.minPower, -robot.minPower, robot.minPower, -robot.minPower);
        else
            robot.drive(- robot.minPower, robot.minPower, -robot.minPower, robot.minPower);

        wait(500);
        robot.brake();
    }
}
