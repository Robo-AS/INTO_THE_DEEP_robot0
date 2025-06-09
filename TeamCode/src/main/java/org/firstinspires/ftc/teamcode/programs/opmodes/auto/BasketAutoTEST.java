package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;


@Autonomous(name = "BasketAutoTEST")
public class BasketAutoTEST extends LinearOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    private Follower follower;
    private Limelight3A limelight;
    public static LLResult result;


    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));

    public static Pose grab1Pose = new Pose(21.038961038961038, 128.57142857142858, Math.toRadians(-9));
    public static Pose score1Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));

    public static Pose grab2Pose = new Pose(21.506493506493506, 131.37662337662337, Math.toRadians(0));
    public static Pose score2Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));

    public static Pose grab3Pose = new Pose(21.038961038961038, 133.71428571428572, Math.toRadians(16));
    public static Pose score3Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));

    public static Pose submersible1Pose = new Pose(60.311688311688314, 94.77922077922078, Math.toRadians(270));
    public static Pose submersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(270));

    public static Pose scoreSubmersible1Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));
    public static Pose scoreSubmersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(-15));
    public PathChain changeHeading;
    private int pathChanged = 0;
    private boolean executedALL = false;
    private boolean executed_1 = false;



    public double y_distance;
    public double x_distance;
    public double targetAngle;
    public double extendoDistance;

    public double CAMERA_ANGLE = 43;
    public double CAMERA_HEIGHT = 350; // mm
    public double LATERAL_OFFSET = 103; // mm
    public double BONUS = 168; // mm





    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(2);
        limelight.start();


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .build();

        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), grab1Pose.getHeading())
                .build();

        PathChain score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1Pose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), score1Pose.getHeading())
                .build();

        PathChain grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(grab2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), grab2Pose.getHeading())
                .build();

        PathChain score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab2Pose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), score2Pose.getHeading())
                .build();

        PathChain grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), grab3Pose.getHeading())
                .build();

        PathChain score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), score3Pose.getHeading())
                .build();

        PathChain submersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3Pose), new Point(submersible1ControlPoint), new Point(submersible1Pose)))
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        changeHeading = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();





        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, scorePreload, true, 1),
                        new FollowPath(follower, grab1, true, 1),
                        new FollowPath(follower, score1, true, 1),
                        new FollowPath(follower, grab2, true, 1),
                        new FollowPath(follower, score2, true, 1),
                        new FollowPath(follower, grab3, true, 1),
                        new FollowPath(follower, score3, true, 1),
                        new FollowPath(follower, submersible1, true, 1),
                        new WaitCommand(1000),


                        new InstantCommand(()->updateLimelight())


                )
        );

        waitForStart();

        while(opModeIsActive()) {
            follower.update();
            CommandScheduler.getInstance().run();


            telemetry.addData("y_distance",y_distance);
            telemetry.addData("x_distance", x_distance);
            telemetry.addData("extendoDistance", extendoDistance);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("EXTENDO TARGET POS:", robot.extendo.getTargetPosition());
            telemetry.addData("EXTENDO CUR POS:", robot.extendo.currentPosition);
            telemetry.addData("EXTEMDO STATE:", robot.extendo.extendoState);
            telemetry.addData("PATH CHANGED?", pathChanged);
            telemetry.addData("ALL?", executedALL);
            telemetry.update();
        }
    }



//    public void buildPath(){
//        robot.limelightCamera.updateLimelight();
//        changeHeading = follower.pathBuilder()
//                .addPath(new BezierPoint(new Point(submersible1Pose)))
//                .setConstantHeadingInterpolation(robot.limelightCamera.getTargetAngle())
//                .build();
//    }

    public void updateLimelight(){
        result=limelight.getLatestResult();
        if (result == null) return;

        double tx = result.getTx();
        double ty = result.getTy();

        y_distance = CAMERA_HEIGHT * Math.tan(Math.toRadians(ty + CAMERA_ANGLE));
        x_distance = Math.sqrt(y_distance * y_distance + CAMERA_HEIGHT * CAMERA_HEIGHT) * Math.tan(Math.toRadians(tx)) - LATERAL_OFFSET;
        extendoDistance = (Math.sqrt((y_distance + BONUS) * (y_distance + BONUS) + x_distance * x_distance) - BONUS) * 1.81;
        targetAngle = -((Math.atan(x_distance / (y_distance + BONUS)))-Math.toRadians(-90));

        changeHeading = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose)))
                .setConstantHeadingInterpolation(targetAngle)
                .build();

        CommandScheduler.getInstance().schedule(
                   new FollowPath(follower, changeHeading, true, 0.5)
        );
    }






}
