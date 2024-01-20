package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * This autonomous mode will leave the specified alliance starting zone
 * by commanding the left & right motors for two seconds before stopping.
 */
@Autonomous
public class Two_Pixel_Closest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors & sensors.
        Hardware hw = new Hardware(hardwareMap);
        double drive_time = 1.25; //amount of time to drive forward in seconds
        double wait_time = 0; //time to delay auto from start of match
        waitForStart();

        // Start timer.
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < wait_time) {
            telemetry.addLine("Delaying auto by 10 sec");
        }
        //reset timer
        timer.reset();
        // Power robot forward for drive_time amount of seconds
        while (timer.seconds() < drive_time) {
            hw.right_motor.setPower(1);
            hw.left_motor.setPower(1);
        }

        //Drop two pixels
        hw.pixel_spear.setPosition(45);
        hw.gusServo.setPosition(0);
        hw.fringServo.setPosition(45);

        // Stop drivetrain
        hw.left_motor.setPower(0);
        hw.right_motor.setPower(0);

    }
}
