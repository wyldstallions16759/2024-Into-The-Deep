package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pinpoint.DriveToPoint;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;


@Autonomous(name="Auto16760")
//@Disabled

public class Auto16760 extends LinearOpMode {

    // Auto State Machine
    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SUBMERSIBLE,
        RETRACT_ARM,
        RELEASE_SPECIMEN,
        DRIVE_TO_GP_1A,
        DRIVE_TO_GP_1B,
        DRIVE_TO_GP_1C,
        DRIVE_TO_GP_1D,
        DRIVE_TO_OBSERVATION_ZONE,
        END
    }

    StateMachine stateMachine;

    Pinpoint pinpoint;
    ArmSubsystem arm;
    WristSubsystem wrist;

    double heading;

    //-----------------------------------------------------------
    // Field poses and arm positions for each state
    //-----------------------------------------------------------

    // ----- State: DRIVE_TO_SUBMERSIBLE -----
    //static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, -180);

    static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -28, 13.6, AngleUnit.DEGREES, 0);
    //static final Pose2D SUBMERSIBLE = new Pose2D(DistanceUnit.INCH, -36, 0, AngleUnit.DEGREES, 180);
    static final int ARM_ELEV_PLACE_SPECIMEN = -900;
    static final int ARM_EXTEND_PLACE_SPECIMEN = 7500;

    // ----- State: PLACE_SPECIMEN -----
    static final int ARM_EXTEND_RELEASE_SPECIMEN = 45000;

    // ----- State: RELEASE_SPECIMEN -----
    // Only operation in this state is to toggle claw, so no need to wait for it to complete

    // ----- State: DRIVE_TO_GP_1A (backup robot, move right past submersible, turn 180)
    static final Pose2D GP1_POSA = new Pose2D(DistanceUnit.INCH, -20, 53.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_GP_1B (go forward)
    static final Pose2D GP1_POSB = new Pose2D(DistanceUnit.INCH, -62, 53.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_GP_1C (move right)
    static final Pose2D GP1_POSC = new Pose2D(DistanceUnit.INCH, -46, 59.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_GP_1D (push game piece to observation zone)
    static final Pose2D GP1_POSD = new Pose2D(DistanceUnit.INCH, -6, 59.6, AngleUnit.DEGREES, 180);

    // ----- State: DRIVE_TO_OBSERVATION_ZONE
    static final Pose2D OBSERVATION_ZONE = new Pose2D(DistanceUnit.INCH, 5, 70, AngleUnit.DEGREES, 90);
    static final int ARM_ELEV_START_POS = 0;
    static final int ARM_EXTEND_START_POS = 0;

    // -----------------------------------
    // Drive and Arm Speeds
    // -----------------------------------
    static final double ARM_ELEVATION_POWER = 1;
    static final double ARM_EXTENSION_POWER = 1;
    static final double DRIVE_SPEED = 0.45;


    @Override
    public void runOpMode() {

        // Initialize Subsystems
        pinpoint = new Pinpoint(this, hardwareMap, telemetry);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        wrist = new WristSubsystem(hardwareMap, telemetry);

        // Initialize state machine
        stateMachine = StateMachine.WAITING_FOR_START;

        // Initialize wrist to starting position
        wrist.wristUp();
        wrist.clawClose();

        // Wait for Autonomous to start
        waitForStart();
        resetRuntime();

        this.getClass();
        // Main loop for Auto
        while (opModeIsActive()) {

            // IMPORTANT: odometry needs to be updated every time through the loop
            pinpoint.update();

            // Send debug info to driver hub
            displayDebugInfo(this.getClass());

            //----------------------------------------------------------
            // State: WAITING_FOR_START
            // Starting state. Move wrist down into position for placing specimen
            //----------------------------------------------------------
            if (stateMachine == StateMachine.WAITING_FOR_START) {
                wrist.wristDown();
                stateMachine = StateMachine.DRIVE_TO_SUBMERSIBLE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_SUBMERSIBLE
            // Actions: Drive to the front of the submersible and elevate and extend arm
            // Next State: DRIVE_TO_SUBMERSIBLE
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE) {
                // In parallel:
                // (a) drive to front of submersible
                // (b) rotate arm to the specified position
                // (c) extend arm to the specified position
                // Move to next state only when all three operations complete
                boolean driveTargetReached = pinpoint.driveTo(SUBMERSIBLE, DRIVE_SPEED, 1);
//                arm.setElevationTarget(ARM_ELEV_PLACE_SPECIMEN);
//                boolean armElevReached = arm.armUp(ARM_ELEVATION_POWER);
//                arm.setExtensionTarget(ARM_EXTEND_PLACE_SPECIMEN);
//                boolean armExtReached = arm.armExtend(ARM_EXTENSION_POWER);

                // If all three conditions are met, move to next state to retract the arm
                if (driveTargetReached) {
                    //if (driveTargetReached && armElevReached && armExtReached) {
                    stateMachine = StateMachine.RETRACT_ARM;
                }

//
//                boolean driveTargetReached = pinpoint.driveTo(SUBMERSIBLE, DRIVE_SPEED , 3);
//                if (driveTargetReached) {
//                    stateMachine = StateMachine.END;
//                }
            }

            //----------------------------------------------------------
            // State: RETRACT_ARM
            // Actions: Retract arm so specimen clips on submersible
            // Next State: RELEASE_SPECIMEN
            //----------------------------------------------------------
            else if (stateMachine == StateMachine.RETRACT_ARM) {
                arm.setExtensionTarget(ARM_EXTEND_RELEASE_SPECIMEN);
                boolean armExtReached = arm.armRetract(ARM_EXTENSION_POWER);

                // If target reached, move to next state to release specimen
                if (armExtReached) {
                    stateMachine = StateMachine.RELEASE_SPECIMEN;
                }
            }

            //----------------------------------------------------------
            // State: RELEASE_SPECIMEN
            // Actions: Open fingers to release specimen
            // Next State: RELEASE_SPECIMEN
            //----------------------------------------------------------
            else if (stateMachine == StateMachine.RELEASE_SPECIMEN) {
                wrist.toggleClaw();
                sleep(500);

                // Don't need to wait for claw to toggle
                stateMachine = StateMachine.DRIVE_TO_GP_1A;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_GP_1
            // Substates: DRIVE_TO_GP_1A, DRIVE_TO_GP_1B, DRIVE_TO_GP_1C, DRIVE_TO_GP_1D
            // Actions: Drive to the first game piece, broken up into 4 substates
            // Next State: PUSH_GP1
            //----------------------------------------------------------
            // Backup, move right, and turn 180 degrees
            else if (stateMachine == StateMachine.DRIVE_TO_GP_1A) {
                boolean driveTargetReached = pinpoint.driveTo(GP1_POSA, DRIVE_SPEED, 0);
                if (driveTargetReached) {
                    stateMachine = StateMachine.DRIVE_TO_GP_1B;
                }
            }
            // Move forward
            else if (stateMachine == StateMachine.DRIVE_TO_GP_1B) {
                boolean driveTargetReached = pinpoint.driveTo(GP1_POSB, DRIVE_SPEED, 0);
                //boolean driveTargetReached = pinpoint.driveTo(new Pose2D(DistanceUnit.INCH, 46, -37.6, AngleUnit.DEGREES, pinpoint.getHeading()), DRIVE_SPEED, 1);
                //heading = pinpoint.getHeading() < 0 ? -180 : 180;
                //boolean driveTargetReached = pinpoint.driveTo(new Pose2D(DistanceUnit.INCH, 46, -37.6, AngleUnit.DEGREES, heading), DRIVE_SPEED, 1);

                if (driveTargetReached) {
                    stateMachine = StateMachine.DRIVE_TO_GP_1C;
                }
            }
            // Move right
            else if (stateMachine == StateMachine.DRIVE_TO_GP_1C) {
                //boolean driveTargetReached = pinpoint.driveTo(GP1_POSC, DRIVE_SPEED, 0);
                boolean driveTargetReached = pinpoint.driveTo(new Pose2D(DistanceUnit.INCH, pinpoint.getX(), -54, AngleUnit.DEGREES, 180), DRIVE_SPEED, 1);
                //heading = pinpoint.getHeading() < 0 ? -180 : 180;
                //boolean driveTargetReached = pinpoint.driveTo(new Pose2D(DistanceUnit.INCH, 46, -54, AngleUnit.DEGREES, heading), DRIVE_SPEED, 1);
                if (driveTargetReached) {
                    stateMachine = StateMachine.DRIVE_TO_GP_1D;
                }
            }
            // Push game piece #1 into Observation Zone
            else if (stateMachine == StateMachine.DRIVE_TO_GP_1D) {
                //boolean driveTargetReached = pinpoint.driveTo(GP1_POSD, DRIVE_SPEED, 0);
                boolean driveTargetReached = pinpoint.driveTo(new Pose2D(DistanceUnit.INCH, 6, pinpoint.getY(), AngleUnit.DEGREES, pinpoint.getHeading()), DRIVE_SPEED, 1);
                //heading = pinpoint.getHeading() < 0 ? -180 : 180;
                //boolean driveTargetReached = pinpoint.driveTo(new Pose2D(DistanceUnit.INCH, 6, -54, AngleUnit.DEGREES, heading), DRIVE_SPEED, 1);
                if (driveTargetReached) {
                    stateMachine = StateMachine.END;
                }
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: open fingers to release specimen
            // Next State: DRIVE_TO_OBSERVATION_ZONE
            //----------------------------------------------------------
            else if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {
                // In parallel:
                // (a) drive to Observation Zone
                // (b) rotate arm to starting position
                // (c) retract arm to starting position
                // Move to next state only when all three operations complete
                boolean driveTargetReached = pinpoint.driveTo(OBSERVATION_ZONE, DRIVE_SPEED, 0);
                arm.setElevationTarget(ARM_ELEV_START_POS);
                boolean armElevReached = arm.armDown(ARM_ELEVATION_POWER);
                arm.setExtensionTarget(ARM_EXTEND_START_POS);
                boolean armExtReached = arm.armRetract(ARM_EXTENSION_POWER);

                // If all three conditions met, robot is parked to done with auto routine
                if (driveTargetReached && armElevReached && armExtReached) {
                    stateMachine = StateMachine.END;
                }
            }

            //----------------------------------------------------------a
            // State: END
            // Actions: Done with auto routine
            //----------------------------------------------------------
            else if (stateMachine == StateMachine.END) {

            }
        }
    }

    private void displayDebugInfo(Class c) {
        telemetry.addData("State: ", stateMachine);
        telemetry.addData("Robot: ", c.getSimpleName());
        telemetry.addData("xPod: ", pinpoint.getEncoderX());
        telemetry.addData("yPod: ", pinpoint.getEncoderY());
        telemetry.addData("Pose X(in): ", pinpoint.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Pose Y(in): ", pinpoint.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Pose Heading(deg): ", pinpoint.getPose().getHeading(AngleUnit.DEGREES));
        //telemetry.addData("yawTolerance: ", pinpoint.getYawTolerance());
        //telemetry.addData("Heading Error: ", Math.toDegrees(pinpoint.getHeadingError()));
        telemetry.addData("ArmElevCurrPosition", arm.getCurrElevPosition());
        telemetry.addData("ArmElevTargetPosition", arm.getElevTargetPosition());
        telemetry.addData("ArmExtCurrPosition", arm.getCurrExtPosition());
        telemetry.addData("ArmExtTargetPosition", arm.getExtTargetPosition());
        telemetry.update();
    }
}