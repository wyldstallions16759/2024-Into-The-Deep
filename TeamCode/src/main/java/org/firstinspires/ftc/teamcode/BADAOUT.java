package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

@Autonomous(name="ABAD")
//@Disabled

// Comment

public class BADAOUT extends LinearOpMode {

    // Auto State Machine
    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_FRONT_SUBMERSIBLE,
        DRIVE_TO_SUBMERSIBLE,
        PLACE_SPECIMEN,
        RELEASE_SPECIMEN,
        DRIVE_TO_OBSERVATION_ZONE,
        POINT1,
        PICKUP,
        END
    }

    //set a bunch of places to go
    static final Pose2D TARGET = new Pose2D(DistanceUnit.INCH, 3, 13, AngleUnit.DEGREES, 90);
    //static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -25, 13.6, AngleUnit.DEGREES, 0);
    static final Pose2D FRONT_SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -28, 13.6, AngleUnit.DEGREES, 0);
    static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -22, 13.6, AngleUnit.DEGREES, 0);
    static final Pose2D POINT2 = new Pose2D(DistanceUnit.INCH, 96, 0, AngleUnit.DEGREES, 180);
    static final Pose2D OBSERVATION = new Pose2D(DistanceUnit.INCH, -5, 30, AngleUnit.DEGREES, 0);
    static final Pose2D POINT1 = new Pose2D(DistanceUnit.INCH, -25, 65, AngleUnit.DEGREES, 90);

    static final int ARM_ROTATION_POSITION = -900;
    static final int ARM_EXTEND_POSITION = 7500;
    static final double ARM_ROTATION_SPEED = 1;
    static final double ARM_EXTENSION_SPEED = 1;

    static final double DRIVE_SPEED = 0.45;


    boolean firstTime = true;
    @Override
    public void runOpMode() {

        // Initialize Pinpoint and Arm
        Pinpoint pinpoint = new Pinpoint(this, hardwareMap, telemetry);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
        WristSubsystem wrist = new WristSubsystem(hardwareMap, telemetry);

        arm = new ArmSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int i = 0;
        boolean firstTime = true;
        // Start state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        //intiialize servos
        wrist.wristUp();
        wrist.clawClose();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        wrist.wristDown();

        // initialize the wrist servo:

        while (opModeIsActive()) {
            //need to update odometry every time it loops
            pinpoint.update();

            //----------------------------------------------------------
            // State: WAITING_FOR_START
            //----------------------------------------------------------

            if (stateMachine == StateMachine.WAITING_FOR_START){
                boolean moving = pinpoint.driveTo(OBSERVATION, DRIVE_SPEED, 0);
                if (!moving) {
                    stateMachine = StateMachine.END;
                }
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_FRONT_SUBMERSIBLE
            // Actions: Drive just short of submersible and elevate and extend arm
            // Next State: DRIVE_TO_SUBMERSIBLE
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_FRONT_SUBMERSIBLE) {

                // Set encoder targets for arm elevation and extension
                if (firstTime){
                    arm.setElevationTarget(ARM_ROTATION_POSITION);
                    arm.setExtensionTarget(ARM_EXTEND_POSITION);
                    firstTime = false;
                }

                // In parallel:
                // (a) drive to front of submersible
                // (b) rotate arm to the specified position
                // (c) extend arm to the specified position
                // Move to next state only when all three operations complete
                boolean driveBusy = pinpoint.driveTo(FRONT_SUBMERSIBLE, DRIVE_SPEED, 0);
                boolean armElevBusy = arm.armUp(ARM_ROTATION_SPEED);
                boolean armExtBusy = arm.armExtend(ARM_EXTENSION_SPEED);

                if (!driveBusy && !armElevBusy && !armExtBusy) {
                    stateMachine = StateMachine.RELEASE_SPECIMEN;
                    firstTime = true;
                     telemetry.addData("Driving to Front Submersible", 0);
                     telemetry.addData("x: ", pinpoint.getCurrentPosition().getX(DistanceUnit.INCH));
                     telemetry.addData("y: ", pinpoint.getCurrentPosition().getY(DistanceUnit.INCH));
                     telemetry.update();
                }
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_SUBMERSIBLE
            // Actions: Move in position to place specimen
            // Next State: RELEASE_SPECIMEN
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE){
                telemetry.addData("Driving to Submersible", 0);
                telemetry.addData("x: ", pinpoint.getCurrentPosition().getX(DistanceUnit.INCH));
                telemetry.addData("y: ", pinpoint.getCurrentPosition().getY(DistanceUnit.INCH));
                telemetry.update();
//                if (firstTime) {
//                    pinpoint.resetPose();
//                    firstTime = false;
//                }
                boolean driveBusy = pinpoint.driveTo(SUBMERSIBLE, DRIVE_SPEED, 0);
                if (!driveBusy) {
                    stateMachine = StateMachine.END;
                    firstTime = true;
                }
            }


            //----------------------------------------------------------
            // State: RELEASE_SPECIMEN
            // Actions: open fingers to release specimen
            // Next State: DRIVE_TO_OBSERVATION_ZONE
            //----------------------------------------------------------
            if (stateMachine == StateMachine.RELEASE_SPECIMEN){
                if (firstTime){
                    arm.setExtensionTarget(-3000);
                    firstTime = false;
                }

                telemetry.addData("state: ", "Release Specimen");

                // In parallel:
                // (a) drive to front of submersible
                // (b) rotate arm to the specified position
                // (c) extend arm to the specified position
                // Move to next state only when all three operations complete
                boolean armExtBusy = arm.armRetract(ARM_EXTENSION_SPEED);
                sleep(3000);
                wrist.toggleClaw();
                sleep(0);
                if (!armExtBusy) {
                    telemetry.update();
                    stateMachine = StateMachine.POINT1;
                    firstTime = true;
                }
            }
            if (stateMachine == StateMachine.POINT1) {
                boolean job1 = pinpoint.driveTo(POINT1,0.3,1);


                if (!job1) {
                    stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
                }
            }


            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: open fingers to release specimen
            // Next State: DRIVE_TO_OBSERVATION_ZONE
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {

                boolean job1 = pinpoint.driveTo(OBSERVATION,0.3,1);

                if (!job1) {
                    stateMachine = StateMachine.END;
                }
            }
            //----------------------------------------------------------
            // State: PICKUP
            // Actions: Done with auto routine
            //----------------------------------------------------------
            if (stateMachine == StateMachine.PICKUP) {
                wrist.toggleClaw();
                sleep(400);
                stateMachine = StateMachine.END;
            }

            //----------------------------------------------------------
            // State: END
            // Actions: Done with auto routine
            //----------------------------------------------------------
            if (stateMachine == StateMachine.END) {
                telemetry.addData("******END*******", 0);
                telemetry.addData("x: ", pinpoint.getCurrentPosition().getX(DistanceUnit.INCH));
                telemetry.addData("y: ", pinpoint.getCurrentPosition().getY(DistanceUnit.INCH));                telemetry.addData("ElevationPos: ", arm.getElevationPos());
                telemetry.addData("ExtensionPos: ", arm.getExtensionPos());
            }

            telemetry.update();
        }
    }

}
