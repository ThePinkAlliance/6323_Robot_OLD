package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Dual Driver Control")
public class TeleDualDriver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware(hardwareMap);

        // Reverse the right side motors. This maleftleftY be wrong for leftleftYour setup.
        // If leftleftYour robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        hw.pixel_dropper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.samMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // The distance calculations are missing the gear ratio between the motor and wheel (two black gears power the robots wheels).
            double left_distance = (hw.left_motor.getCurrentPosition() / 40 / 28) * 3.5 * 3.14;
            double right_distance = (hw.right_motor.getCurrentPosition() / 40 /28) * 3.5 * 3.14;

            boolean clawUp = gamepad2.dpad_up; // Lower power;
            boolean clawDown = gamepad2.dpad_down;

            double left_y = gamepad1.left_stick_y;
            double right_y = gamepad1.right_stick_y;

            hw.right_motor.setPower(right_y);
            hw.left_motor.setPower(left_y);

            if(clawUp) {
                hw.pixel_dropper.setPower(-0.7);
                hw.samMotor.setPower(0.7);
            } else if(clawDown){
                hw.pixel_dropper.setPower(0.7);
                hw.samMotor.setPower(-0.7);
            } else {
                hw.pixel_dropper.setPower(0.0);
                hw.samMotor.setPower(0.0);
            }
            //close arm
            if(gamepad2.left_bumper){
                hw.gusServo.setPosition(45);
                hw.fringServo.setPosition(0);
                //open arm
            } else if(gamepad2.right_bumper){
                hw.gusServo.setPosition(0);
                hw.fringServo.setPosition(45);
            }
            //open finger
            if(gamepad2.a){
                hw.silly.setPosition(0);
            }
            //close finger
            else if(gamepad2.b){
                hw.silly.setPosition(45);
            }

            //raise pixel spear
            if(gamepad2.y){
                hw.pixel_spear.setPosition(0);
            }
            //lower pixel spear
            else if(gamepad2.x){
                hw.pixel_spear.setPosition(45);
            }

            telemetry.addData("left_distance", hw.left_motor.getCurrentPosition());
            telemetry.addData("right_distance", hw.right_motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
