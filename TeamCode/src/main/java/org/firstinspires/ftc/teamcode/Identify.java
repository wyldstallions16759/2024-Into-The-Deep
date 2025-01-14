package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Identify{
    // Identify what robot this will be run on.
    public static enum Robot{
        FTC16760,
        FTC28147
    }

    private HardwareMap map;
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rf;
    public DcMotor rb;
    public Robot bot;

    public Identify(HardwareMap m){
        map = m;
        lf = map.get(DcMotor.class, "frontLeft");
        lb = map.get(DcMotor.class, "backLeft");
        rf = map.get(DcMotor.class, "frontRight");
        try {
            rb = map.get(DcMotor.class, "backRight");
            bot = Robot.FTC16760;
        }
        catch (Exception e){
            rb = map.get(DcMotor.class, "backRight28147");
            bot = Robot.FTC28147;
        }
    }

}