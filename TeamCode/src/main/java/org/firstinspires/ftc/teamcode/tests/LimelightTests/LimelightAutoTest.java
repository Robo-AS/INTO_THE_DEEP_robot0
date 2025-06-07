package org.firstinspires.ftc.teamcode.tests.LimelightTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;

@Config
@TeleOp(name = "LimelightAutoTest")
public class LimelightAutoTest extends CommandOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    public GamepadEx gamepadEx;

    private Limelight3A limelight;
    public static LLResult result;

    public double y_distance;
    public double x_distance;
    public double targetAngle;
    public double extendoDistance;

    public double CAMERA_ANGLE = 43;
    public double CAMERA_HEIGHT = 350; // mm
    public double LATERAL_OFFSET = 103; // mm
    public double BONUS = 168; // mm

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private PathChain changeHeading;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        changeHeading = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(startPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(2);
        limelight.start();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();

//        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(
//                        () -> {
//                            updateLimelight();
//                            CommandScheduler.getInstance().schedule(
//                                    new SequentialCommandGroup(
//                                            new FollowPath(follower, changeHeading, true, 0.5)
//
//
//                                    )
//                            );
//                        }
//                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> updateLimelight()),
                                new FollowPath(follower, changeHeading, true, 0.5)
                        )
                )
        );

    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();

        robot.loop();
        robot.intake.loopBlue();
        robot.extendo.loop(gamepadEx.getLeftY());
        robot.lift.loop();
        robot.arm.loopTeleOp();

//        result = limelight.getLatestResult();
//
//        if (result == null) {
//            telemetry.addData("NULL", "NULL");
//            telemetry.update();
//            return;
//        }
//
//        double tx = result.getTx();
//        double ty = result.getTy();
//
//        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
//        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
//        extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81;
//        targetAngle = -(Math.atan(x_distance / (y_distance + BONUS)));


//        telemetry.addData("tx", tx);
//        telemetry.addData("ty", ty);
        telemetry.addData("y_distance", y_distance);
        telemetry.addData("x_distance", x_distance);
        telemetry.addData("extendoDistance", extendoDistance);
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("EXTENDO TARGET POS:", robot.extendo.getTargetPosition());
        telemetry.addData("EXTENDO CUR POS:", robot.extendo.currentPosition);
        telemetry.addData("EXTEMDO STATE:", robot.extendo.extendoState);
        telemetry.update();
    }


    public void updateLimelight(){
        result=limelight.getLatestResult();
        if (result == null) return;

        double tx = result.getTx();
        double ty = result.getTy();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
        extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81;
        targetAngle = -(Math.atan(x_distance / (y_distance + BONUS)));

        changeHeading = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(startPose)))
                .setConstantHeadingInterpolation(targetAngle)
                .build();
    }

}
