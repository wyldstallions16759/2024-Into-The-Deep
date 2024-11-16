package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Andres' arm example teleop", group="Linear OpMode")

public class AndresArmTeleop extends LinearOpMode{
    //declare variables
    private ElapsedTime runtime = new ElapsedTime();

    private AndresWristSubsystem wristSubsystem;
    @Override
    public void runOpMode(){
         wristSubsystem = new AndresWristSubsystem(hardwareMap, telemetry);

         waitForStart();
         runtime.reset();

         while (opModeIsActive()){
             // use operator controls to move arm
             double wrist = gamepad2.left_stick_x;
             double twist = gamepad2.right_stick_x;

             // operator control to close fingers
             boolean closeFingers = gamepad2.right_bumper;

             // operator control to move wrist to preset positions:
             boolean upPreset = gamepad2.dpad_up;
             boolean downPreset = gamepad2.dpad_down;

             // move to preset, if applicable
             if (upPreset){
                 wristSubsystem.wristUp();
             }

             if (downPreset){
                 wristSubsystem.wristNormal();
             }

             // for the twist value, the range must be modified
             double twistServoPosition = wristSubsystem.joystickToRange(twist);

             // move wrist to the position
             wristSubsystem.move(wrist, twistServoPosition);

             if (closeFingers){
                 wristSubsystem.close();
             }
             else{
                 wristSubsystem.open();
             }

             // update telemetry
             telemetry.update();
         }
    }
}
