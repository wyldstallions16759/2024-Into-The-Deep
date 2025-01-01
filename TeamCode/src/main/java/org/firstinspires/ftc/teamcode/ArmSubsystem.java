package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    // Motors for elevating and extending the arm
    private DcMotor elevation = null;
    private DcMotor extension = null;

    // Use to print to the driver hub
    private Telemetry telemetry;

    // Save targets for elevating and extending the arm
    private int elevationTarget;
    private int extensionTarget;


    // Constructor
    public ArmSubsystem(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // Initialize arm motors
        // Reset the motor encoders so they have a value of 0 at startup
        elevation = hwMap.get(DcMotor.class, "Elevation");
        elevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extension = hwMap.get(DcMotor.class, "Extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the Elevation target position for subsequent armUp and armDown calls
    // Target is remembered until this function is called again
    public void setElevationTarget(int newPosition){
        elevationTarget = newPosition;
    }

    // Set the Extension target position for subsequent armExtend and armRetract calls
    // Target is remembered until this function is called again
    public void setExtensionTarget(int newPosition){
        extensionTarget = newPosition;
    }

    //------------------------------------------------------------------------------------
    // Move arm up at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setElevationTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armUp(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (elevation.getCurrentPosition() > elevationTarget) {
            elevation.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        elevation.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Move arm down at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setExtensionTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armDown(double power) {
        // Target not yet reached
        if (elevation.getCurrentPosition() < elevationTarget) {
            // Due to the way the arm motor is oriented, positive speed moves it down
            elevation.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        elevation.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Extend arm out at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setExtensionTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armExtend(double power) {
        // Target not yet reached
        if (extension.getCurrentPosition() < extensionTarget){
            telemetry.addData("reached position", 1);
            // Positive speed extends the arm
            extension.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        extension.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Retract arm at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setExtensionTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armRetract(double power) {
        // Target not yet reached
        if (extension.getCurrentPosition() > extensionTarget){
            telemetry.addData("reached position", 1);
            // Negative speed retracts the arm
            extension.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        extension.setPower(0);
        return true;
    }

    //-----------------------------
    // Getter functions
    //-----------------------------
    public int getCurrElevPosition () {
        return elevation.getCurrentPosition();
    }

    public int getElevTargetPosition () {
        return elevationTarget;
    }

    public int getCurrExtPosition () {
        return extension.getCurrentPosition();
    }

    public int getExtTargetPosition () {
        return extensionTarget;
    }
}
