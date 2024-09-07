package org.firstinspires.ftc.teamcode.autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.concurrent.TimeUnit;

@Autonomous
public class HuskyLens_Red_Backdrop extends LinearOpMode {
    com.qualcomm.hardware.dfrobot.HuskyLens huskyLens;
    Hardware hw;
    private final int READ_PERIOD = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);

        hw.samMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.pixel_dropper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.samMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //hw.Color.enableLed(true);

        huskyLens = hardwareMap.get(com.qualcomm.hardware.dfrobot.HuskyLens.class, "huskylens");

        waitForStart();

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the right_motor Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */

        telemetry.update();
        waitForStart();

        //STAGE 0 - Auto Routine Init
        com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks; // object to store blocks from the husky
        ElapsedTime timer; //object to store a general timer
        ElapsedTime timeout; //object to store a timer to give up the husky routine after 5 seconds of failed attempts to locate the team prop
        int propLocation = -1; //0 left, 1 center, 2 right
        int redSetpoint = 650;

        //STAGE 1 - Find Team Prop and move to the left/center/right spike

        huskyLens.selectAlgorithm(com.qualcomm.hardware.dfrobot.HuskyLens.Algorithm.OBJECT_TRACKING); //switch to obj recognition for team prop
        //start the 5 second timeout timer
        timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        //search blocks for the team prop (id 1)
        while (propLocation < 0 && timeout.seconds() < 5) { //while the prop is not found within 5 second timeout...
            blocks = huskyLens.blocks(); // update the list of blocks with the objects currently seen by the husky
            for (int i = 0; i < blocks.length; i++) {
                if (blocks[i].id == 1) { //if team prop found, detirmine if positioned on left/center/right spike
                    if (blocks[i].x <= 105) { //left
                        propLocation = 0;
                        break;
                    } else if (blocks[i].x >= 90 && blocks[i].x <= 195) { //center
                        propLocation = 1;
                        break;
                    } else if (blocks[i].x >= 195) { //right
                        propLocation = 2;
                        break;
                    }
                }
            }
            //error message if no prop is found, list all found blocks to screen
            telemetry.addData("Auto Status", "INFO: Searching for prop... ");
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            telemetry.update();
            //wait 0.5 seconds before retrying
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.5);
        }
        //check if timer had expired before an object was recognised
        if (timeout.seconds() > 5 && propLocation < 0) {
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 3) {
                telemetry.addData("Auto Status", "TIMEOUT: Prop not found in 5 sec, Default to Left. ");
                telemetry.update();
            }
            propLocation = 0;
        }
        //if prop was found within 5 seconds, continue with routine
        telemetry.addData("Auto Status", "PROP FOUND: " + propLocation);
        telemetry.update();
        //move forward quickly to leave the starting area
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 1.25) { //give 0.5 sec to move off of center spike
            hw.right_motor.setPower(-0.7);
            hw.left_motor.setPower(-0.7);
        }
        //move forward slowly until the color sensor detects the center spike mark
        while(hw.Color.red() < redSetpoint) {
            hw.right_motor.setPower(-0.3);
            hw.left_motor.setPower(-0.3);
            telemetry.addData("Auto Status", "INFO: Looking for red... " + hw.Color.red());
            telemetry.update();
        }
        hw.right_motor.setPower(0);
        hw.left_motor.setPower(0);
        //turn towards team prop
        if (propLocation == 0) {
            //move backwards briefly to make space for the truses
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.5) { //give 0.5 sec to move off of center spike
                hw.right_motor.setPower(.3);
                hw.left_motor.setPower(.3);
            }
            while(hw.Color.red() < redSetpoint) { //then start checking for left spike
                hw.right_motor.setPower(-0.0375);
                hw.left_motor.setPower(0.3);
                telemetry.addData("Auto Status", "INFO: Looking for red... " + hw.Color.red());
                telemetry.update();
            }
            //drive off line so pixel hits line
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.25) { //give 0.5 sec to move off of center spike
                hw.right_motor.setPower(-0.05);
                hw.left_motor.setPower(-0.4);
            }
        }
        if (propLocation == 1) { //prop location 1 (center) has already been reached, just move off line a little
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.5) { //give 0.5 sec to move off of center spike
                hw.right_motor.setPower(-.3);
                hw.left_motor.setPower(-.3);
            }
        }
        else if (propLocation == 2) {
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.5) { //give 0.5 sec to move off of center spike
                hw.right_motor.setPower(0.4);
                hw.left_motor.setPower(0.05);
            }
            while(hw.Color.red() < redSetpoint) { //then start checking for right spike
                hw.right_motor.setPower(0.3);
                hw.left_motor.setPower(0.0325);
                telemetry.addData("Auto Status", "INFO: Looking for red... " + hw.Color.red());
                telemetry.update();

            }
            //drive off line so pixel hits line
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.5) { //give 0.5 sec to move off of center spike
                hw.right_motor.setPower(-0.4);
                hw.left_motor.setPower(-0.05);
            }
        }
        hw.left_motor.setPower(0);
        hw.right_motor.setPower(0);
        //drop the pixel, wait for robot to stop first
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 0.5) {
            hw.right_motor.setPower(0);
            hw.left_motor.setPower(0);
        }
        hw.pixel_spear.setPosition(0.14);
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 3) {
            hw.right_motor.setPower(0);
            hw.left_motor.setPower(0);
        }
        hw.pixel_spear.setPosition(1);
        //realign with center spike mark
        if (propLocation == 0) { //left
            do {
                hw.right_motor.setPower(-0.075);
                hw.left_motor.setPower(-0.3);
                telemetry.addData("Auto Status", "INFO: Looking for red... " + hw.Color.red());
                telemetry.update();
            } while(hw.Color.red() < redSetpoint);
        }
        else if (propLocation == 1) {//prop location 1 (center)
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.5) { //give 0.5 sec to move off of center spike
                hw.right_motor.setPower(.3);
                hw.left_motor.setPower(.3);
            }
        }
        else if (propLocation == 2) { //right

            do {
                hw.right_motor.setPower(-0.3);
                hw.left_motor.setPower(-0.0325);
                telemetry.addData("Auto Status", "INFO: Looking for red... " + hw.Color.red());
                telemetry.update();

            } while(hw.Color.red() < redSetpoint);
        }
        //stop before turn towards backboard
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 0.25) {
            hw.right_motor.setPower(0);
            hw.left_motor.setPower(0);
        }
        //turn right towards backboard, left spike mark needs shorter turn time
        if(propLocation == 0) {
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.7) {
                hw.right_motor.setPower(-.5);
                hw.left_motor.setPower(.5);
            }
        }
        else {
            timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            while (timer.seconds() < 0.7) {
                hw.right_motor.setPower(-.5);
                hw.left_motor.setPower(.5);
            }
        }
        //stop before move towards backboard
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 0.25) {
            hw.right_motor.setPower(0);
            hw.left_motor.setPower(0);
        }
        //move forward towards backboard
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 1.6) {
            hw.right_motor.setPower(-5);
            hw.left_motor.setPower(-5);
        }
        //stop before droping pixel
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 0.25) {
            hw.right_motor.setPower(0);
            hw.left_motor.setPower(0);
        }
        //release 2nd pixel
        hw.fringServo.setPosition(1);
        hw.gusServo.setPosition(0);
        //stop while pixel drops
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (timer.seconds() < 3) {
            hw.right_motor.setPower(0);
            hw.left_motor.setPower(0);
        }


    }
}
