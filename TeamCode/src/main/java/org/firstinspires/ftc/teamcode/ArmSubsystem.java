package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem {

    private DcMotor Elevation = null;
    private DcMotor Extension = null;

    // here is the telemetry:
    private Telemetry telemetry;

    private int speed;
    // Constructor
    public ArmSubsystem(HardwareMap hwMap, Telemetry telemetry) {

        //set telemetry:
        this.telemetry = telemetry;
        this.speed = 4;
        //actually define elevate and extend
        Elevation = hwMap.get(DcMotor.class, "Elevation");
        Extension = hwMap.get(DcMotor.class, "Extension");
        //set modes to run using the encoder
        Elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // Move arm up until the newPosition reached
    public boolean armUp(int newPosition) {
        telemetry.addData("current pos: ", Elevation.getCurrentPosition());
        telemetry.addData("target pos: ", newPosition);
        Elevation.setTargetPosition(newPosition);
        Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevation.setPower(speed);
        if (Elevation.isBusy()){
            return false;
        }
        Elevation.setPower(0);
        Elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
    }

    // Move arm down until newPosition reached
    public boolean armDown(int newPosition) {
        telemetry.addData("current pos: ", Elevation.getCurrentPosition());
        telemetry.addData("target pos: ", newPosition);
        Elevation.setTargetPosition(newPosition);
        Elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Elevation.setPower(-speed);
        if (Elevation.isBusy()){
            return false;
        }
        Elevation.setPower(0);
        Elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}

//telemetry code for later
// telemetry.addData("Elevation Encoder: ", "%d", Elevation.getCurrentPosition());
//telemetry.addData("Extension Encoder: ", "%d", Extension.getCurrentPosition());