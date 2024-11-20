package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="USE_THIS_TELEOP", group="Linear OpMode")

public class ArmTestTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    ArmSubsystem arm = new ArmSubsystem(hardwareMap, telemetry);
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        boolean armBusy = false;
        while(opModeIsActive()){
            boolean up = gamepad2.dpad_down;
            boolean down = gamepad2.dpad_up;

            if (!armBusy) {
                if (up) {
                    armBusy = arm.armUp(1000);
                }
                if (down) {
                    armBusy = arm.armDown(1000);
                }
            }
        }
    }
}
