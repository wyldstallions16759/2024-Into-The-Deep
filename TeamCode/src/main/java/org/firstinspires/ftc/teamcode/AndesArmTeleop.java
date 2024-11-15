package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Mutli-joint arm", group="Linear OpMode")

public class AndesArmTeleop extends LinearOpMode{
    //declare variables
    private ElapsedTime runtime = new ElapsedTime();

    private AndresWristSubsystem wristSubsystem;
    @Override
    public void runOpMode(){
         wristSubsystem = new AndresWristSubsystem(hardwareMap, telemetry);

         waitForStart();
         runtime.reset();

         while (opModeIsActive()){
             //use operator controls to move arm
             double wrist = gamepad2.left_stick_x;
             double twist = gamepad2.right_stick_x;

             //operator control to close fingers
             boolean closeFingers = gamepad2.right_bumper;

             //make -1 -> 0 and 1 -> 1
             double wristServoPosition = wristSubsystem.calculateWristPosition(wrist);
             double twistServoPosition = wristSubsystem.calculateTwistPosition(twist);

             //move wrist to the position
             wristSubsystem.moveTo(wristServoPosition, twistServoPosition);

             if (closeFingers){
                 wristSubsystem.close();
             }
             else{
                 wristSubsystem.open();
             }

            //update telemetry
             telemetry.update();
         }
    }
}
