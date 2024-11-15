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
        //to be calibrated later.
        //due to the calculations performed from the "calculate____Position()",
        // this is not necessary ATM. It's a good idea to fill these out when you have a chance
        // and replace the need for a calculate___Position() method.

        claw.scaleRange(0.52,0.55); //this will need to be calibrated to the Servo's positionnnn.
    }

    // moveTo() method:
    // accepts two doubles within range [0-1]
    // this sets the servo position to that, based on the current ranges (set within this class)
    public void moveTo(double wristPosition, double twistPosition){
        // These values (wristPosition and twistPosition) will be able to be
        // calculated in some method I will write later.

        base.setPosition(wristPosition);
        twist.setPosition(twistPosition);

        telemetry.addData("wristpos:", wristPosition);
        telemetry.addData("twistpos:",twistPosition);
    }

    public double calculateWristPosition(double input){
        // this method accepts a double, expected to be within the range [-1, 1]
        // it turns it into a number in the range [0.12, 0.8] that can be used for the servos.
        return (input+1)*2.0/3 + (2.0/15);
    }

    public double calculateTwistPosition(double input){
        // accepts a double, expected to be in range [-1, 1]
        // it turns it into a number [0.05916666666,0.23666666666]
        return ((input/3)+1)*(0.1775);
    }

    // open() and close() methods:
    // they will open and close the hand to preset positions.

    public void open(){
        claw.setPosition(0);
    }

    public void close(){
        claw.setPosition(1);
    }
}
