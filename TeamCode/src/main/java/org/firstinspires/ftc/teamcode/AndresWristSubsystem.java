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

        twist.scaleRange(71.0/400,(71.0/300)+(71.0/2400));
        // TWIST range should be [71/600, 71/300]
        // this creates 180 degrees of motion. It is possible to enlarge this to allow for a full
        // rotation, because twist servo is a 5-turn servo and not limited to 300 degrees.
        // in between these two values is 71/400, which is a "normal" position. It faces forward.
        // an input of "0.5" will give the "normal" position.

        // UPDATE: make 71.0/300 the 'normal' position
        // keep 135deg range, Min: -45deg from normal
        // Max: 90deg past normal
        // if a range of (71/600) to (71/300)=142/600 is 180degrees, 180 degree range = 71/600
        // 45 is 1/4 of 180. (71/2400)
        //NORMAL should be 71/300. MAX should be (71/300) + (71/2400)
        // MIN should be 71/400
        // 71/300 is around 129/200 in the [0,1] range.

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

        // set the twist position. It will not have to have anything special because it will default back to its normal position.
        // its normal position is currently 0.5 (halfway between the two extremes). If a change
        // is made to allow it to have 270 degrees of rotation, a change will need to be made here to make it default to the new
        // default position.
        // A CHANGE HAS BEEN MADE. It now must rotate 135 degrees with a base at 129/200

        // Manipulate the twist range:
        // Turn it into a range of [0, 529/400]
        // or not...

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
