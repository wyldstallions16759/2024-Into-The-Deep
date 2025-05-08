package org.firstinspires.ftc.teamcode.Pinpoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

public class Pinpoint {

    // Odometry pods x and y offsets - MUST BE FILLED IN
//    final static double XOFFSET = mmPerInch * 4.25;
//    final static double YOFFSET = mmPerInch * 4.0625;
    final static double XOFFSET = mmPerInch * 6.6;
    final static double YOFFSET = mmPerInch * 6.75;


    private GoBildaPinpointDriver odo;
    private DriveToPoint nav;

    public Pinpoint(LinearOpMode opMode, HardwareMap hwMap, Telemetry telemetry, double driveP, double driveD,double driveAcceleration,double driveTolerance, double yawP, double yawD,double yawAcceleration,double yawTolerance) {

        // Initialize the Pinpoint
        initPinpoint(hwMap);

        // Initiaize DriveToPoint
        nav = new DriveToPoint(opMode);
        nav.initializeMotors();
        nav.setXYCoefficients(driveP, driveD, driveAcceleration, INCH, driveTolerance);
        nav.setYawCoefficients(yawP, yawD , yawAcceleration, DEGREES, yawTolerance);
    }

    public void initPinpoint(HardwareMap hwMap) {
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(XOFFSET, YOFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public boolean driveTo(Pose2D targetPosition, double power, double holdTime) {
        return nav.driveTo(odo.getPosition(), targetPosition, power, holdTime);

    }
    public void update() {
        odo.update();
    }
    public void setFix(boolean fix) {
        nav.setFix(fix);
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }

    public double getX() {return getPose().getX(INCH);}
    public double getY() {return getPose().getY(INCH);}
    public double getHeading() {return getPose().getHeading(DEGREES);}


    public double getEncoderX() {
        return odo.getEncoderX();
    }
    public double getEncoderY() {
        return odo.getEncoderY();
    }

    public double getDiff() { return nav.diff(); }
    public double getHeadingError() {return nav.hError();}
    public double getYawTolerance() {return nav.getYawTolerance(); }
    public boolean xOutOfBounds() {return nav.xOutOfBounds();}
    public boolean yOutOfBounds() {return nav.yOutOfBounds();}

}