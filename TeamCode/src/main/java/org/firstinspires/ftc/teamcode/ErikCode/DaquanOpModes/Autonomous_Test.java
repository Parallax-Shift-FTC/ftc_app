package org.firstinspires.ftc.teamcode.ErikCode.DaquanOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Brown on 11/13/2017.
 */
@Autonomous(name = "Erik's Auto Test", group = "Daquan Erik")
public class Autonomous_Test extends LinearOpMode{
    Daquan_Hardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //Sets up the robot and waits for the start button to be pressed
        robot = new Daquan_Hardware(hardwareMap, telemetry, false);

        robot.UpdateColor();

        if(robot.red > robot.green || robot.blue > robot.green)
        {
            if(robot.red > robot.blue)
            {
                robot.drive(.2,-.2,.2,-.2);
                sleep(1000);
                robot.drive(0,0,0,0);
            }
            else {
                robot.drive(-.2, .2, -.2, .2);
                sleep(1000);
                robot.drive(0, 0, 0, 0);
            }
        }
        else
            robot.drive(0,0,0,0);

        telemetry.addData("red", robot.red);
        telemetry.addData("blue", robot.blue);
        telemetry.addData("green", robot.green);
        telemetry.update();
    }
}
