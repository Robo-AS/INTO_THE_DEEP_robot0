package org.firstinspires.ftc.teamcode.tests.LimelightTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@TeleOp(name = "LimelightTestPythonSnap", group = "Tests")
public class LimelightTestPythonSnap extends LinearOpMode {
    private Limelight3A limelight;
//    public static LLResult result;


    public double y_distance;
    public double x_distance;
    public double targetAngle;
    public double extendoDistance;

    public double CAMERA_ANGLE = 43;
    public double CAMERA_HEIGHT = 350; //in mm, relative to the front of the intake -30mm(for the sample height)
    public double LATERAL_OFFSET = 103; //in mm (the intake is 103mm to the right of the camera)
    public double BONUS = 168; //in mm (the distance from the front of the intake to the robot center)



    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);


        double[] inputs = {0.0, 1.0};
        limelight.updatePythonInputs(inputs);


        limelight.start();



        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length >= 1) {
                double tx = pythonOutputs[0];
                double ty = pythonOutputs[1];

                y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
                x_distance = Math.sqrt(y_distance*y_distance + CAMERA_HEIGHT*CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
                extendoDistance = (Math.sqrt((y_distance+BONUS)*(y_distance+BONUS) + x_distance*x_distance) - BONUS) * 1.81;
                targetAngle = Math.atan2(x_distance, y_distance + BONUS);

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("y_distance", y_distance);
                telemetry.addData("x_distance", x_distance);
                telemetry.addData("extendoDistance", extendoDistance);
                telemetry.addData("targetAngle", Math.toDegrees(targetAngle));
                telemetry.update();

            } else {
                telemetry.addData("Status", "No valid Python output");
                telemetry.update();
            }
        }
    }

}
