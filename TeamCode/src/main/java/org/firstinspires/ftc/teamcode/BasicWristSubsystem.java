// A basic subsystem to control the wrist/claw
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicWristSubsystem {

    //some constants.
    private final double CLAW_OPEN = 0;
    private final double CLAW_CLOSED = 1;


    //define important variables
    private Telemetry telemetry;
    private Servo clawServo;

    //make a toggle method:
    private boolean clawOpen = false;

    public BasicWristSubsystem(HardwareMap hwmap, Telemetry tele){
        // Initialize the motors and telemetry
        this.clawServo = hwmap.get(Servo.class, "RightFinger");

        // set the telemetry so we can use it later:
        this.telemetry = tele;

        // set servo ranges:
        this.clawServo.scaleRange(0,1); //this will need to be changed soon.

        this.clawServo.setPosition(CLAW_CLOSED);
    }

    public void open(){
        // tells the servo to open.
        // tells the computer that it is open now.
        this.clawServo.setPosition(CLAW_OPEN);
        clawOpen = true;
    }

    public void close(){
        // tells the servo to close.
        // tells the computer that it is closed now.
        this.clawServo.setPosition(CLAW_CLOSED);
        clawOpen = false;
    }

    public void toggleClaw(){
        // pretty basic, if its open, close it. if its closed, open it.
        if (clawOpen){
            this.close();
        }
        else{
            this.open();
        }
    }
}
