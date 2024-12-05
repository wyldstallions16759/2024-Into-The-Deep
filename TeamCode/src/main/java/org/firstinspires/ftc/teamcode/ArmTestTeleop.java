package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestArmTeleOp", group="Linear OpMode")

public class ArmTestTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    ArmSubsystem arm;
    public void runOpMode() {
        arm = new ArmSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        int i = 0;
        boolean firstTime = true;
        while(opModeIsActive()){
            telemetry.addData("running", i);
            if (firstTime){
                arm.setExtensionTarget(1000);
                firstTime = false;
            }
            boolean armBusy = arm.armRetract();
            if (!armBusy) {
                sleep(1000);
                break;
            }
            i++;
            telemetry.update();
        }
    }
}
