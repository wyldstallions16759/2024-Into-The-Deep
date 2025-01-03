package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    // Motors for elevating and extending the arms
    private DcMotor elevationR = null;
    private DcMotor extensionR = null;
    private DcMotor elevationL = null;
    private DcMotor extensionL = null;

    // Use to print to the driver hub
    private Telemetry telemetry;

    // Save targets for elevating and extending the arm
    private int elevationTargetR;
    private int extensionTargetR;
    private int elevationTargetL;
    private int extensionTargetL;


    // Constructor
    public ArmSubsystem(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // Initialize arm motors
        // Reset the motor encoders so they have a value of 0 at startup
        elevationR = hwMap.get(DcMotor.class, "rRot");
        elevationR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevationR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevationR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionR = hwMap.get(DcMotor.class, "rExt");
        extensionR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevationL = hwMap.get(DcMotor.class, "lRot");
        elevationL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevationL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevationL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionL = hwMap.get(DcMotor.class, "lExt");
        extensionL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the Elevation target position for subsequent armUp and armDown calls
    // Target is remembered until this function is called again
    public void setElevationTargetR(int newPosition){
        elevationTargetR = newPosition;
    }

    public void setElevationTargetL(int newPosition){
        elevationTargetL = newPosition;
    }

    // Set the Extension target position for subsequent armExtend and armRetract calls
    // Target is remembered until this function is called again
    public void setExtensionTargetR(int newPosition){
        extensionTargetR = newPosition;
    }

    public void setExtensionTargetL(int newPosition){
        extensionTargetL = newPosition;
    }

    //------------------------------------------------------------------------------------
    // Move arm up at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setElevationTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armUpR(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (elevationR.getCurrentPosition() > elevationTargetR) {
            elevationR.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        elevationR.setPower(0);
        return true;
    }

    public boolean armUpL(double power) {
        // Target not yet reached
        // Due to the way the arm motor is oriented, negative power moves it up
        if (elevationL.getCurrentPosition() > elevationTargetL) {
            elevationL.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        elevationL.setPower(0);
        return true;
    }
    //------------------------------------------------------------------------------------
    // Move arm down at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setExtensionTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armDownR(double power) {
        // Target not yet reached
        if (elevationR.getCurrentPosition() < elevationTargetR) {
            // Due to the way the arm motor is oriented, positive speed moves it down
            elevationR.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        elevationR.setPower(0);
        return true;
    }

    public boolean armDownL(double power) {
        // Target not yet reached
        if (elevationL.getCurrentPosition() < elevationTargetL) {
            // Due to the way the arm motor is oriented, positive speed moves it down
            elevationL.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        elevationL.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Extend arm out at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setExtensionTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armExtendR(double power) {
        // Target not yet reached
        if (extensionR.getCurrentPosition() < extensionTargetR){
            telemetry.addData("reached position", 1);
            // Positive speed extends the arm
            extensionR.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        extensionR.setPower(0);
        return true;
    }

    public boolean armExtendL(double power) {
        // Target not yet reached
        if (extensionL.getCurrentPosition() < extensionTargetL){
            telemetry.addData("reached position", 1);
            // Positive speed extends the arm
            extensionL.setPower(power);
            return false;
        }
        // Target reached - current position >= newPosition. Stop motor and return true
        extensionL.setPower(0);
        return true;
    }

    //------------------------------------------------------------------------------------
    // Retract arm at the specified power (0 - 1.0]
    // Return true if target is reached; false if not yet reached.
    // setExtensionTarget() MUST be called before calling this function.
    //------------------------------------------------------------------------------------
    public boolean armRetractR(double power) {
        // Target not yet reached
        if (extensionR.getCurrentPosition() > extensionTargetR){
            telemetry.addData("reached position", 1);
            // Negative speed retracts the arm
            extensionR.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        extensionR.setPower(0);
        return true;
    }

    public boolean armRetractL(double power) {
        // Target not yet reached
        if (extensionL.getCurrentPosition() > extensionTargetL){
            telemetry.addData("reached position", 1);
            // Negative speed retracts the arm
            extensionL.setPower(-power);
            return false;
        }
        // Target reached - current position <= newPosition. Stop motor and return true
        extensionL.setPower(0);
        return true;
    }

    //-----------------------------
    // Getter functions
    //-----------------------------
    public int getCurrElevPositionR () {
        return elevationR.getCurrentPosition();
    }

    public int getCurrElevPositionL () {
        return elevationL.getCurrentPosition();
    }

    public int getElevTargetPositionR () {
        return elevationTargetR;
    }

    public int getElevTargetPositionL () {
        return elevationTargetL;
    }

    public int getCurrExtPositionR () {
        return extensionR.getCurrentPosition();
    }

    public int getCurrExtPositionL () {
        return extensionL.getCurrentPosition();
    }

    public int getExtTargetPositionR () {
        return extensionTargetR;
    }

    public int getExtTargetPositionL () {
        return extensionTargetL;
    }
}