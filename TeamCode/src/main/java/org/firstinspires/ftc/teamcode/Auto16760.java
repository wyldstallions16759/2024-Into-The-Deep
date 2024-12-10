package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

@Autonomous(name="Auto16760and28147bad")
@Disabled

// Comment

public class Auto16760 extends LinearOpMode {

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
    static final Pose2D TARGET = new Pose2D(DistanceUnit.INCH, 5, 50, AngleUnit.DEGREES, 90);
    static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 0);
    static final Pose2D POINT2 = new Pose2D(DistanceUnit.INCH, 96, 0, AngleUnit.DEGREES, 180);
    static final Pose2D OBSERVATION = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);
    static final Pose2D POINT1 = new Pose2D(DistanceUnit.INCH, 24, 48, AngleUnit.DEGREES, 90);

    static final int ARM_UP_POSITION = 1000;

    boolean firstTime = true;
    @Override
    public void runOpMode() {

        // Initialize Pinpoint and Arm
        Pinpoint pinpoint = new Pinpoint(this, hardwareMap, telemetry);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
        Teleop16760and28147 map = new Teleop16760and28147();


        // Start state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            //need to update odometry every time it loops
            pinpoint.update();

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
                boolean job1 = pinpoint.driveTo(TARGET,0.3,1);
                Pose2D pose = pinpoint.getCurrentPosition();
                telemetry.addData("X: ", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y: ", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading: ", pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Target:", ARM_UP_POSITION);
                if (job1) {
                    stateMachine = StateMachine.END;
                }
            }

            //----------------------------------------------------------
            // State: PLACE_SPECIMEN
            // Actions: retract arm until target location reached
            // Next State: RELEASE_SPECIMEN
            //----------------------------------------------------------

            if (stateMachine == StateMachine.PLACE_SPECIMEN){
                //pull arm down
                //release claw
            }

            //----------------------------------------------------------
            // State: RELEASE_SPECIMEN
            // Actions: open fingers to release specimen
            // Next State: DRIVE_TO_OBSERVATION_ZONE
            //----------------------------------------------------------
            if (stateMachine == StateMachine.RELEASE_SPECIMEN){
                // Release specimen
                boolean job1 = pinpoint.driveTo(POINT1,0.3,0.1);
                boolean job2 = pinpoint.driveTo(POINT2,0.3,0.);
                if (job1 & job2) {
                    stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
                }
                }

            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: drive to observation zone and park
            // Next State: END
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {
                boolean job1 = pinpoint.driveTo(OBSERVATION,0.3,100);
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
