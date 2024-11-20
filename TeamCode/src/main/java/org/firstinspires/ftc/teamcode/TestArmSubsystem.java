package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.Sigma;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic Arm System Test TeleOp", group="000TESTING")

public class TestArmSubsystem extends LinearOpMode{
    //declare variables
    private ElapsedTime runtime = new ElapsedTime();

    private BasicWristSubsystem wristSubsystem;

    @Override
    public void runOpMode(){
        wristSubsystem = new BasicWristSubsystem(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        boolean wasXDown = false;

        while (opModeIsActive()){
            // set controls
            boolean isXDown = gamepad2.x;

            if (isXDown && !wasXDown){
                wristSubsystem.toggleClaw();
            }

            // set wasXDown at the end
            wasXDown = isXDown;
        }
    }
}