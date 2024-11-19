package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.Pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

@Autonomous(name="Auto16760and28147")
//@Disabled

// Comment

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

    //set a bunch of places to go
    static final Pose2D TARGET = new Pose2D(DistanceUnit.MM,50, 50, AngleUnit.DEGREES, 0);
    static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 0);
    static final Pose2D OBSERVATION = new Pose2D(DistanceUnit.INCH, 96, 0, AngleUnit.DEGREES, 90);


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

//            pinpoint.driveTo(TARGET, 0.3, 0);
            boolean Job1 = true;
            if (Job1) {
                stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
            }

            while (pinpoint.getXpos() <100000) {
                telemetry.addData("X position:", pinpoint.getXpos());
                telemetry.addData("Y position:", pinpoint.getYpos());
                telemetry.update();
            }
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
//                pinpoint.driveTo(TARGET, 0.3, 5);
//                pinpoint.driveTo(TARGET, 0.3, 5);
                // (b) rotate arm to ARM_UP_MAX_POSITION
                // (c) extend arm to EXT_MAX_POSITION
                // When all three conditions met, move to next state (PLACE_SPECIMEN)

                int x = 1;
                if (x == 1) {
                    stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
                }

            }

            //----------------------------------------------------------
            // State: PLACE_SPECIMEN
            // Actions: retract arm until target location reached
            // Next State: RELEASE_SPECIMEN
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {

            }
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

        }
    }
}
