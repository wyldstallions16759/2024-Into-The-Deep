package org.firstinspires.ftc.teamcode.Pinpoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

public class Pinpoint {

    // Odometry pods x and y offsets - MUST BE FILLED IN
    final double XOFFSET = 0;
    final double YOFFSET = 0;

    private GoBildaPinpointDriver odo;
    private DriveToPoint nav;

    public Pinpoint(LinearOpMode opMode, HardwareMap hwMap, Telemetry telemetry) {

        // Initialize the Pinpoint
        initPinpoint(hwMap);

        // Initiaize DriveToPoint
        nav = new DriveToPoint(opMode);
        nav.initializeMotors();
        nav.setXYCoefficients(0.01, 0, 2.0, MM, 12);
        nav.setYawCoefficients(2, 0.5, 2.0, DEGREES, 2);
    }

    private void initPinpoint(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(XOFFSET, YOFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        return nav.driveTo(currentPosition, targetPosition, power, holdTime);
    }
}