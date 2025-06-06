package org.firstinspires.ftc.teamcode.tests.LimelightTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "LimelightTest", group = "Tests")
public class LimelightTest extends LinearOpMode {
    private Limelight3A limelight;
    public static LLResult result;

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
        limelight.pipelineSwitch(2);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            result = limelight.getLatestResult();

            if(result == null){
                telemetry.addData("NULL", "NULL");
                telemetry.update();
                return;
            }

            double tx = result.getTx();
            double ty = result.getTy();

            y_distance = CAMERA_HEIGHT * Math.tan((Math.toRadians(ty + CAMERA_ANGLE)));
            x_distance = Math.sqrt(y_distance*y_distance + CAMERA_HEIGHT*CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
            extendoDistance = (Math.sqrt((y_distance+BONUS)*(y_distance+BONUS) + x_distance*x_distance) - BONUS) * 1.81;
            targetAngle = Math.atan(x_distance/(y_distance + BONUS));




            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("y_distance", y_distance);
            telemetry.addData("x_distance", x_distance);
            telemetry.addData("extendoDistance", extendoDistance);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.update();

        }
    }
}
