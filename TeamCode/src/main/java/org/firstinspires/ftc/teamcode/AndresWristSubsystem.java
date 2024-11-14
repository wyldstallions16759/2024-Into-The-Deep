// A subsystem for controlling the absurdly complex wrist that Andres made.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AndresWristSubsystem {
    // declare important versions of variables from the teleop/autoop modes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // declare variables for the three servo motors in the wrist:
    private Servo base;
    private Servo twist;
    private Servo claw;
    public AndresWristSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        //assign the local (this class) hardwareMap and telemetry variables
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // upon initialization, define the servo motor variables
        base = hardwareMap.get(Servo.class, "wrist");
        twist = hardwareMap.get(Servo.class, "twist");
        claw = hardwareMap.get(Servo.class, "hand");

        //use the scaleRange() method of the Servo class to set the Servo ranges:
        base.scaleRange(0,1);
        twist.scaleRange(0,1);
        claw.scaleRange(0,1); //to be calibrated later.
    }

    // moveTo() method:
    // accepts two doubles within range [0-1]
    // this sets the servo position to that, based on the current ranges (set within this class)
    public void moveTo(double wristPosition, double twistPosition){
        // These values (wristPosition and twistPosition) will be able to be
        // calculated in some method I will write later.

        base.setPosition(wristPosition);
        twist.setPosition(twistPosition);
    }
}
