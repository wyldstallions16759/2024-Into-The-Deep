package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    // Constructor
    public ArmSubsystem(HardwareMap hwMap, Telemetry telemetry) {

        // Initialize motors
    }


    // Move arm up until the newPosition reached
    public boolean armUp(int newPosition) {
        return true;
    }

    // Move arm down until newPosition reached
    public boolean armDown(int newPosition) {
        return true;

    }

    // Extend arm out until the newPosition reached
    public boolean armExtend(int newPosition) {
        return true;

    }

    // Extend arm out until the newPosition reached
    public boolean armRetract(int newPosition) {
        return true;

    }

    public boolean fingersOpen() {
        return true;

    }

    public boolean fingersClose() {
        return true;

    }
}
