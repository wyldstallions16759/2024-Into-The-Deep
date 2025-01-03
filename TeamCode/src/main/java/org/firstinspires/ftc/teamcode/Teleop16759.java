/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop16759")
//@Disabled
public class Teleop16759 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftExtension = null;
    private DcMotor rightExtension = null;
    private DcMotor leftRotate = null;
    private DcMotor rightRotate = null;

    private Servo intake1 = null;
    private Servo intake2 = null;

    @Override
    public void runOpMode() {

        ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
        Pinpoint pinpoint = new Pinpoint(this, hardwareMap, telemetry);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        leftExtension = hardwareMap.get(DcMotor.class, "lExt");
        rightExtension = hardwareMap.get(DcMotor.class, "rExt");
        leftRotate = hardwareMap.get(DcMotor.class, "lRot");
        rightRotate = hardwareMap.get(DcMotor.class, "rRot");

        //get servos too!
        intake1 = hardwareMap.get(Servo.class, "in1");
        intake2 = hardwareMap.get(Servo.class, "in2");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //set zero power behaviors:
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pinpoint.update();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // arm controls:
            boolean rotateUpLeft = gamepad2.dpad_up;
            boolean rotateUpRight = gamepad2.y;

            boolean rotateDownLeft = gamepad2.dpad_down;
            boolean rotateDownRight = gamepad2.a;

            // arm rotate controls
            double doubleMin = 0.3;

            double extendLeft = -gamepad2.left_stick_y; //left extension is reversed.
            double extendRight = gamepad2.right_stick_y;

            int extendLeftInt = 0; //-1, 0, 1 depending on stick input
            int extendRightInt = 0;

            if (extendLeft > doubleMin){
                extendLeftInt = 1;
            }
            else if (extendLeft < -doubleMin){
                extendLeftInt = -1;
            }

            if (extendRight > doubleMin){
                extendRightInt = 1;
            }
            else if (extendRight < -doubleMin){
                extendRightInt = -1;
            }

            // intake controls
            double intake_in = gamepad2.left_trigger;
            double intake_out = gamepad2.right_trigger;

            boolean take_in = intake_in > doubleMin;
            boolean take_out = intake_out > doubleMin;

            // slowmode:
            boolean slow = gamepad1.right_trigger > doubleMin;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            int slowfactor = 5;

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (slow) {
                leftFrontPower /= slowfactor;
                rightFrontPower /= slowfactor;
                leftBackPower /= slowfactor;
                rightBackPower /= slowfactor;
            }

            // This is test code:

            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // calculate angles and stuff
            double leftRotAngle = -leftRotate.getCurrentPosition() * Math.PI/10000;
            double rightRotAngle = -rightRotate.getCurrentPosition() * Math.PI/10000;
            telemetry.addData("leftrotate:", leftRotAngle);
            telemetry.addData("rightrotate", rightRotAngle);

            double maxLeftExtLen = 40/Math.cos(leftRotAngle);
            double maxRightExtLen = 40/Math.cos(rightRotAngle);

            // Extensions:
            double extensionPower = 0.5;
            double leftExtLen = leftExtension.getCurrentPosition()*(63.0/4300);
            telemetry.addData("leftExtLen",leftExtLen);
            telemetry.addData("maxleftextlen",maxLeftExtLen);
            // Left Extension:
            if (extendLeftInt == 1 && leftExtLen < maxLeftExtLen){//maxLeftExtLen){
                leftExtension.setPower(extensionPower);
            }
            else if (extendLeftInt == -1 || leftExtLen > maxLeftExtLen){
                leftExtension.setPower(-extensionPower);
            }
            else{
                leftExtension.setPower(0);
            }

            telemetry.addData("Left Extend: ", extendLeftInt);

            // Right Extension
            double rightExtLen = -rightExtension.getCurrentPosition()*(63.0/4300);

            telemetry.addData("rightExtLen",rightExtLen);
            telemetry.addData("maxrightextlen",maxRightExtLen);

            if (extendRightInt == 1 || rightExtLen > maxRightExtLen){//maxRightExtLen){
                rightExtension.setPower(extensionPower);
            }
            else if (extendRightInt == -1 && rightExtLen < maxRightExtLen){
                rightExtension.setPower(-extensionPower);
            }
            else{
                rightExtension.setPower(0);
            }

            telemetry.addData("Right Extend: ", extendRightInt);

            //arm rotate:
            //left arm
            double armRotatePower = 0.5;
            String rlupdate = "None";
            if (rotateUpLeft){
                leftRotate.setPower(armRotatePower);
                rlupdate = "Up";
            }
            else if (rotateDownLeft && leftRotate.getCurrentPosition() > -5000){
                leftRotate.setPower(-armRotatePower);
                rlupdate = "Down";
            }
            else{
                leftRotate.setPower(0);
            }

            telemetry.addData("Left Rotate: ", rlupdate);

            //right arm

            String rrupdate = "None";
            if (rotateUpRight){
                rightRotate.setPower(armRotatePower);
                rrupdate = "Up";
            }
            else if (rotateDownRight){
                rightRotate.setPower(-armRotatePower);
                rrupdate = "Down";
            }
            else{
                rightRotate.setPower(0);
            }

            telemetry.addData("Right Rotate: ", rrupdate);

            // Now, do intake!
            double intakespeed = 1;
            if (take_in){
                intake1.setPosition(intakespeed);
                intake2.setPosition(1-intakespeed);
            }
            else if (take_out){
                intake1.setPosition(1-intakespeed);
                intake2.setPosition(intakespeed);
            }
            else{
                intake1.setPosition(0.5);
                intake2.setPosition(0.5);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Right Encoder:", rightRotate.getCurrentPosition());
            telemetry.addData("Left Encoder:", leftRotate.getCurrentPosition());
            telemetry.addData("Right Ext Encoder:", rightExtension.getCurrentPosition());
            telemetry.addData("Left Ext Encoder:", leftExtension.getCurrentPosition());
            telemetry.addData("Odometry Pod X:", pinpoint.getEncoderX());
            telemetry.addData("Odometry Pod Y:", pinpoint.getEncoderY());
            telemetry.addData("ArmElevCurrPositionL", arm.getCurrElevPositionL());
            telemetry.addData("ArmElevTargetPositionL", arm.getElevTargetPositionL());
            telemetry.addData("ArmExtCurrPositionL", arm.getCurrExtPositionL());
            telemetry.addData("ArmExtTargetPositionL", arm.getExtTargetPositionL());
            telemetry.update();
        }
    }
}
