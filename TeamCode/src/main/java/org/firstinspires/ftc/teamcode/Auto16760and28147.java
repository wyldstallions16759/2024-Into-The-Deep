package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

@Autonomous(name="Auto16760and28147")
//@Disabled

public class Auto16760and28147 extends LinearOpMode {

    // Auto State Machine
    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SUBMERSIBLE,
        PLACE_SPECIMEN,
        RELEASE_SPECIMEN,
        DRIVE_TO_OBSERVATION_ZONE,
        END
    }

    boolean firstTime = true;

    @Override
    public void runOpMode() {

        // Initialize Pinpoint and Arm
        Pinpoint pinpoint = new Pinpoint(this, hardwareMap, telemetry);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);

        // Start state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            //----------------------------------------------------------
            // State: WAITING_FOR_START
            //----------------------------------------------------------
            if (stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_SUBMERSIBLE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_SUBMERSIBLE
            // Actions:
            // Next State: PLACE_SPECIMEN
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE) {
                // In parallel:
                // (a) drive to front of SUBMERSIBLE
                // (b) rotate arm to ARM_UP_MAX_POSITION
                // (c) extend arm to EXT_MAX_POSITION
                // When all three conditions met, move to next state (PLACE_SPECIMEN)

            }

            //----------------------------------------------------------
            // State: PLACE_SPECIMEN
            // Actions: retract arm until target location reached
            // Next State: RELEASE_SPECIMEN
            //----------------------------------------------------------
            if (stateMachine == StateMachine.PLACE_SPECIMEN){

            }

            //----------------------------------------------------------
            // State: RELEASE_SPECIMEN
            // Actions: open fingers to release specimen
            // Next State: DRIVE_TO_OBSERVATION_ZONE
            //----------------------------------------------------------
            if (stateMachine == StateMachine.RELEASE_SPECIMEN){
                // Release specimen
                // TODO
                stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: drive to observation zone and park
            // Next State: END
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {
                stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
            }


            //----------------------------------------------------------
            // State: END
            // Actions: Done with auto routine
            //----------------------------------------------------------
            if (stateMachine == StateMachine.END) {

            }

            telemetry.update();
        }
    }
}
