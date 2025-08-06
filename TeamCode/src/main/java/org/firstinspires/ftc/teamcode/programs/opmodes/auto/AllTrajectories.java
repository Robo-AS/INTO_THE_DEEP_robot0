//package org.firstinspires.ftc.teamcode.programs.opmodes.auto;
//
//import com.seattlesolvers.solverslib.command.CommandScheduler;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;
//
//@Disabled
//@Autonomous(name = "AllTrajectories")
//public class AllTrajectories extends LinearOpMode {
//    private final NEWRobot robot = NEWRobot.getInstance();
//    private Follower follower;
//
//
//    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
//    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));//1
//
//    public static Pose grab1Pose = new Pose(21.038961038961038, 128.57142857142858, Math.toRadians(-17));//2
//    public static Pose score1Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));//3
//
//    public static Pose grab2Pose = new Pose(21.506493506493506, 131.37662337662337, Math.toRadians(0));//4
//    public static Pose score2Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));//5
//
//    public static Pose grab3Pose = new Pose(21.038961038961038, 133.71428571428572, Math.toRadians(15));//6
//    public static Pose score3Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));//7
//
//    public static Pose submersible1Pose = new Pose(60.311688311688314, 94.77922077922078, Math.toRadians(270));//8
//    public static Pose submersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(270));
//
//    public static Pose scoreSubmersible1Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));       //9
//    public static Pose scoreSubmersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(-15));
//
//
//    public static Pose submersible2Pose = new Pose(63.350649350649356, 94.77922077922078, Math.toRadians(275));             //10
//    public static Pose submersible2ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(275));
//
//    public static Pose scoreSubmbersible2Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));      //11
//    public static Pose scoreSubmersible2ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(-15));
//
//    public static Pose submersible3Pose = new Pose(63.350649350649356, 94.77922077922078, Math.toRadians(265));     //12
//    public static Pose submersible3ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(265));
//
//    public static Pose scoreSubmbersible3Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));      //13
//    public static Pose scoreSubmersible3ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(-15));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CommandScheduler.getInstance().reset();
//
//
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//
//        PathChain scorePreload = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
//                .build();
//
//        PathChain grab1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(preloadPose), new Point(grab1Pose)))
//                .setLinearHeadingInterpolation(preloadPose.getHeading(), grab1Pose.getHeading())
//                .build();
//
//        PathChain score1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(grab1Pose), new Point(score1Pose)))
//                .setLinearHeadingInterpolation(grab1Pose.getHeading(), score1Pose.getHeading())
//                .build();
//
//        PathChain grab2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score1Pose), new Point(grab2Pose)))
//                .setLinearHeadingInterpolation(score1Pose.getHeading(), grab2Pose.getHeading())
//                .build();
//
//        PathChain score2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(grab2Pose), new Point(score2Pose)))
//                .setLinearHeadingInterpolation(grab2Pose.getHeading(), score2Pose.getHeading())
//                .build();
//
//        PathChain grab3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score2Pose), new Point(grab3Pose)))
//                .setLinearHeadingInterpolation(score2Pose.getHeading(), grab3Pose.getHeading())
//                .build();
//
//        PathChain score3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(grab3Pose), new Point(score3Pose)))
//                .setLinearHeadingInterpolation(grab3Pose.getHeading(), score3Pose.getHeading())
//                .build();
//
////        PathChain submersible1 = follower.pathBuilder()
////                .addPath(new BezierCurve(new Point(score3Pose), new Point(submersible1ControlPoint), new Point(submersible1Pose)))
////                .setLinearHeadingInterpolation(score3Pose.getHeading(), submersible1Pose.getHeading())
////                .build();
//        PathChain submersible1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(score3Pose), new Point(submersible1ControlPoint), new Point(submersible1Pose)))
//                .setTangentHeadingInterpolation()
//                .setReversed(false)
//                .setZeroPowerAccelerationMultiplier(16)
//                .build();
//
////        PathChain scoreSubmersible1 = follower.pathBuilder()
////                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
////                .setLinearHeadingInterpolation(submersible1Pose.getHeading(), scoreSubmersible1Pose.getHeading())
////                .build();
//        PathChain scoreSubmersible1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
//                .setZeroPowerAccelerationMultiplier(14)
//                .build();
//
//        PathChain submersible2 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scoreSubmersible1Pose), new Point(submersible2ControlPoint), new Point(submersible2Pose)))
//                .setLinearHeadingInterpolation(scoreSubmersible1Pose.getHeading(), submersible2Pose.getHeading())
//                .setReversed(false)
//                .setZeroPowerAccelerationMultiplier(16)
//                .build();
//
//        PathChain scoreSubmersible2 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(submersible2Pose), new Point(scoreSubmersible2ControlPoint), new Point(scoreSubmbersible2Pose)))
//                .setLinearHeadingInterpolation(submersible2Pose.getHeading(), scoreSubmbersible2Pose.getHeading())
//                .setReversed(true)
//                .setZeroPowerAccelerationMultiplier(14)
//                .build();
//
//        PathChain submersible3 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scoreSubmbersible2Pose), new Point(submersible3ControlPoint), new Point(submersible3Pose)))
//                .setLinearHeadingInterpolation(scoreSubmbersible2Pose.getHeading(), submersible3Pose.getHeading())
//                .setReversed(false)
//                .setZeroPowerAccelerationMultiplier(16)
//                .build();
//
//
//        PathChain scoreSubmersible3 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(submersible3Pose), new Point(scoreSubmersible3ControlPoint), new Point(scoreSubmbersible3Pose)))
//                .setLinearHeadingInterpolation(submersible3Pose.getHeading(), scoreSubmbersible3Pose.getHeading())
//                .setReversed(true)
//                .setZeroPowerAccelerationMultiplier(14)
//                .build();
//
//
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new FollowPath(follower, scorePreload, true, 1),
//                        new FollowPath(follower, grab1, true, 1),
//                        new FollowPath(follower, score1, true, 1),
//                        new FollowPath(follower, grab2, true, 1),
//                        new FollowPath(follower, score2, true, 1),
//                        new FollowPath(follower, grab3, true, 1),
//                        new FollowPath(follower, score3, true, 1),
//                        new FollowPath(follower, submersible1, true, 1),
//                        new FollowPath(follower, scoreSubmersible1, true, 1),
//                        new FollowPath(follower, submersible2, true, 1),
//                        new FollowPath(follower, scoreSubmersible2, true, 1),
//                        new FollowPath(follower, submersible3, true, 1),
//                        new FollowPath(follower, scoreSubmersible3, true, 1)
//
//
//                )
//        );
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            follower.update();
//            CommandScheduler.getInstance().run();
//        }
//    }
//}
