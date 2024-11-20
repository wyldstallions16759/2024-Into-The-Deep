// A subsystem for controlling the absurdly complex wrist that Andres made.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AndresWristSubsystem {
    // important constants for servo positions:

    public final double WRIST_NORMAL_POSITION = 10.0/13;
    public final double WRIST_UP_POSITION = 0;

    // WRIST_ROTATE_DIVISOR
    // when rotating the wrist, the wristPosition variable is
    // incremented or decremented by the joystick position divided by this value.
    // MUST BE CALIBRATED
    public final int WRIST_ROTATE_DIVISOR = 500;
    public final double TWIST_NORMAL_POSITION = 0.5;


    // declare important versions of variables from the teleop/autoop modes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // declare variables for the three servo motors in the wrist:
    private Servo base;
    private Servo twist;
    private Servo claw;

    // declare servo progress variables
    // this will be used to make the base wrist "sticky".
    // it will move when pushed, but not default back to its normal position.
    private double wristPosition;

    public AndresWristSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        //assign the local (this class) hardwareMap and telemetry
        // variables to be used throughout the class
        this.hardwareMap = hardwareMap; // not currently used.
        this.telemetry = telemetry;

        // the subsystem will be initialized upon construction.
        // upon initialization, define the servo motor variables
        // NOTES:
        // "base" is a 300 degree rotation servo.
        // "twist" is a 5-turn servo.
        // "claw" is a 5-turn servo.

        base = hardwareMap.get(Servo.class, "wrist");
        twist = hardwareMap.get(Servo.class, "twist");
        claw = hardwareMap.get(Servo.class, "hand");

        //use the scaleRange() method of the Servo class to set the Servo ranges:
        base.scaleRange(2.0/15,1);
        // BASE range should be [2/15, 1]. 0.8 is a "normal" position, facing straight down.
        // going past 0.8 is possible, but only a little bit (because of how the servo range lined up)
        // 0.0 is a secondary "normal" position, facing straight up.
        // an input of 10.0/13 (0.76923076923) should give the "normal" position.

        // NOTE: must be 10.0/13 because integer division (10/13) would return 0

        twist.scaleRange(71.0/400,213.0/800);

        // TWIST range should be between [71/400, 213/800]
        // This *should* give a range of 135 degrees.
        // A normal position *should* be 129/200 of the range [71/400, 213/800]

        // if this doesn't work, just revert the branch. I definitely should have commited when I had it working,
        // but it's a little too late for that :/ Hopefully this will work.


        claw.scaleRange(0.53,0.55); //this will need to be calibrated to the Servo's position.

        // further initialize instance variables

        // initialize sticky variables (see declaration of this.wristPosition)
        this.wristPosition = WRIST_NORMAL_POSITION;
    }

    public void move(double wristChange, double twistPosition){
        // accepts two double parameters.
        // wristChange indicates the position of the joystick that is tasked with moving the wrist.
        // it is a double within the range [-1, 1].

        // twistPosition is a position for the twist joint. It is a double between [0, 1].
        // wrist joint is sticky, being set to a variable that is incremented and decremented within
        // this method. It will hold its position until reset.

        // twist joint is NOT sticky. It will only hold its position as long as it is receiving
        // input from the twistPosition variable in this method.


        // Manipulate the twist range:
        // The wrist must be adjusted to allow for the new "normal" position at 129/200
        // The movement will be linear for each side, but in order to have different distances on each
        // side of the center, some will need to move faster.

        // All inputs below 0.5 will be Case 1, and all inputs above or equal to 0.5 will be Case 2.
        // Case 1 will be linear with other points in Case 1, but not with those in Case 2.

        // If you want to switch the order of the controls, uncomment the line below:
        // twistPosition = 1 - twistPosition;
        // (I think that should work).

        if (twistPosition < 0.5){
            // Case 1:
            // something must be MULTIPLIED because 0 remains equal to 0.
            // However, 0.5 becomes 129/200
            // to turn 0.5 into 129/200, you would multiply by (129/200)/0.5 = (129/200)/(1/2) = (129/200)*2 = 129/100.
            twistPosition *= 129.0/100;
        }
        else{
            // Case 2:
            // first, we subtract 0.5 from twistPosition. Now, it is within a range [0, 0.5].
            // 0.5 will become 1, and 0 will become 129/200. To do this, we multiply first by some scalar,
            // and then add a constant to it. The constant added will be 129/200.
            // (because 0k = 0, and then 0 must become 129/200.) The scalar will be
            // the number that, when multiplied by 0.5, is equal to 1 - (129/200). That's 200/200 - 129/200 = 71/200.
            // 0.5k = 71/200. Divide both sides by 0.5. k = (71/200)/(1/2) = 71/100.

            twistPosition -= 0.5;
            twistPosition *= 71.0/100;
            twistPosition += 129.0/200;
        }
        twist.setPosition(twistPosition);

        // wrist position will need to be incremented by some amount, and then assigned to the servo position.
        // the amount the wrist will be incremented will be calculated by the wrist change (a double
        // in the range [-1, 1]) and a divisor. ****The divisor will need to be calibrated.****
        // wrist is a 300 degree servo motor, so even a change of 0.1 (10%) is 30 degrees (significant)

        // first, check if the wristChange is ****EXTREMELY**** close to 0 and should be ignored.
        double abs = Math.abs(wristChange);

        // if abs is large enough, run the setPosition statements.
        // this will prevent issues with controllers that aren't perfect.

        if (abs > 0.1){
            // the larger WRIST_ROTATE_DIVISOR is, the smaller wristIncrement will be.
            // small values are good because it can be used for fine adjustments. Presets will be
            // available for normal (vertical) positions.

            double wristIncrement = wristChange/WRIST_ROTATE_DIVISOR;

            // increment wristPosition:
            this.wristPosition += wristIncrement;
            // make sure the wristPosition is within the range [0,1]
            this.wristPosition = Math.max(0,Math.min(1,this.wristPosition));
            // picks the smaller of (position) and 1, then the bigger one of the result.
            // this forces it to be in the range [0,1]

            base.setPosition(this.wristPosition);
            // print to telemetry
            telemetry.addData("Wrist Position: ",this.wristPosition);
            telemetry.addData("Wrist Change:", wristIncrement);
        }

        // print information to telemetry:

        telemetry.addData("Wrist Set Position:", this.wristPosition);
        telemetry.addData("Wrist Twist Set Position: ",twistPosition);

        // check to see if there is a difference between actual position and set position
        // (just for my curiosity)
        telemetry.addData("Wrist Actual Position:", base.getPosition());
        telemetry.addData("Wrist Twist Actual Position:", twist.getPosition());

    }

    public double joystickToRange(double input){
        // accepts a double, expected to be in range [-1, 1]
        // it turns it into a double in the range [0, 1].
        // add 1 to turn it into a range [0, 2].
        // divide by 2 to turn the range into [0, 1]

        // can be used for the twistPosition in the move() method.

        return (input+1)/2;
    }

    // open() and close() methods:
    // they will open and close the hand to preset positions.

    public void open(){
        claw.setPosition(0);
    }

    public void close(){
        claw.setPosition(1);
    }

    // preset positions:

    public void wristNormal(){
        // wristNormal moves wrist to a normal (down-facing) position

        this.wristPosition = WRIST_NORMAL_POSITION;

        // assign the position to the motor in case move() isn't being called regularly:
        base.setPosition(this.wristPosition);
    }
    public void wristUp(){
        // wristNormal moves wrist to an up-facing position

        this.wristPosition = WRIST_UP_POSITION;

        // assign the position to the motor in case move() isn't being called regularly:
        base.setPosition(this.wristPosition);
    }
}
