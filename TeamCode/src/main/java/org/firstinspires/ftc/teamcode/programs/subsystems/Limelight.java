package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class Limelight extends SubsystemBase {
    private static Limelight instance = null;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public Limelight3A limelight;

    public double y_distance;
    public double x_distance;
    public double targetAngle;
    public double extendoDistance;

    public double CAMERA_ANGLE = 43;
    public double CAMERA_HEIGHT = 350; //in mm, relative to the front of the intake -30mm(for the sample height)
    public double LATERAL_OFFSET = -103; //in mm (the intake is 103mm to the right of the camera)
    public double BONUS = 168; //in mm (the distance from the front of the intake to the robot center)


    public double TUNNING_CONSTANT_EXTENDO = 30;


    public void initializeHardware(final HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void initializeBLUE(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void initializeRED(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void updateLimelightColorTresh(){
        LLResult result = limelight.getLatestResult();
        if (result == null) return;

        double tx = result.getTx();
        double ty = result.getTy();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) + LATERAL_OFFSET;
        extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81;
        targetAngle = -(Math.atan(x_distance / (y_distance + BONUS)));

        Globals.extendoDistance = (int)extendoDistance;
    }


//    public void upadateLimelightPythonSnap(){
//        LLResult result = limelight.getLatestResult();
//
//        double[] pythonOutputs = result.getPythonOutput();
//        if (pythonOutputs != null && pythonOutputs.length >= 1) {
//            double tx = pythonOutputs[0];
//            double ty = pythonOutputs[1];
//
//            y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
//            x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
//            extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81;
//            targetAngle = Math.atan2(x_distance, y_distance + BONUS);
//
//            Globals.extendoDistance = (int)extendoDistance;
//        }
//
//    }


    public void upadateLimelightPythonSnap(){
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            double tx = result.getTx();
            double ty = result.getTy();

            y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
            x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) + LATERAL_OFFSET;
            extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81 - TUNNING_CONSTANT_EXTENDO;
            targetAngle = -(Math.atan(x_distance / (y_distance + BONUS)));

            Globals.extendoDistance = (int)extendoDistance;

        }

    }


}
