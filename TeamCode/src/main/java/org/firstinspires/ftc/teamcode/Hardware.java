package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class Hardware {
    public DcMotor right_motor, left_motor, pixel_dropper, samMotor;
    public Servo gusServo, fringServo, silly, pixel_spear;
    public RevColorSensorV3 Color;

    public Hardware(HardwareMap hardwareMap) {
        right_motor = hardwareMap.dcMotor.get("leftMotor");
        left_motor = hardwareMap.dcMotor.get("rightMotor");
        pixel_dropper = hardwareMap.dcMotor.get("flintLockwoodMotor"); //Left Arm Motor
        samMotor = hardwareMap.dcMotor.get("samMotor"); //Right Arm Motor
        pixel_spear = hardwareMap.get(Servo.class, "pixel_spear"); //Rear-mounted pre-loaded pixel
        gusServo = hardwareMap.get(Servo.class,"gus"); // Right Claw Servo
        fringServo = hardwareMap.get(Servo.class, "fring"); // Left Claw Servo
        silly = hardwareMap.get(Servo.class, "silly"); //Airplane Launcher Pin Servo
        Color = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
    }
}
