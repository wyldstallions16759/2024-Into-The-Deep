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
        while(opModeIsActive()){
            telemetry.addData("running", i);
            boolean armBusy = arm.armUp(-1000);
            if (!armBusy){
                sleep(100000);
                break;
            }
            i++;
            telemetry.update();
        }
    }
}
