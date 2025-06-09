package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight extends SubsystemBase {
    private static Limelight instance = null;


    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public Limelight3A limelight;
    public static LLResult result;

    public double y_distance;
    public double x_distance;
    public double targetAngle;
    public double extendoDistance;

    public double CAMERA_ANGLE = 43;
    public double CAMERA_HEIGHT = 350;
    public double LATERAL_OFFSET = 103;
    public double BONUS = 168;

    public void initializeHardware(final HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void initialize(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(2);
        limelight.start();
    }

    public void updateLimelight(){
        result = limelight.getLatestResult();
        if (result == null) return;

        double tx = result.getTx();
        double ty = result.getTy();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
        extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81;
        targetAngle = -(Math.atan(x_distance / (y_distance + BONUS)));
    }

    public double getTargetAngle(){
        return targetAngle;
    }

}
