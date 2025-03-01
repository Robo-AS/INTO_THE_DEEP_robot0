package org.firstinspires.ftc.teamcode.tests.AUTO_TESTS;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

//@Config
//@Autonomous(name = "BasketAutoTestTrajectories_LINEAR")
public class BasketAutoTestTrajectories_LINEAR extends CommandOpMode {
    private final Robot robot = Robot.getInstance();

    private Follower follower;

    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));

    public static Pose grab1Pose = new Pose(28, 120.8, Math.toRadians(0));
    public static Pose score1Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose grab2Pose = new Pose(28, 130, Math.toRadians(0));
    public static Pose score2Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose grab3Pose = new Pose(28, 134, Math.toRadians(14));
    public static Pose score3Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose submersible1Pose = new Pose(59.434720608912535, 94.92507791217606, Math.toRadians(270));
    public static Pose submersible1ControlPoint = new Pose(57.328765941667605, 117.09057925187031, Math.toRadians(270));

    public static Pose scoreSubmersible1Pose = new Pose(16.5, 129.5, Math.toRadians(-45));
    public static Pose scoreSubmersible1ControlPoint = new Pose(57.328765941667605, 117.09057925187031, Math.toRadians(-45));

    public static Pose submersible2Pose = new Pose(63.88062490642961, 95.15907287520326, Math.toRadians(275));
    public static Pose submersible2ControlPoint = new Pose(60.83869038707582, 118.02655910397917, Math.toRadians(275));

    public static Pose scoreSubmbersible2Pose = new Pose(16.5, 129.5, Math.toRadians(-45));
    public static Pose scoreSubmersible2ControlPoint = new Pose(60.83869038707582, 118.02655910397917, Math.toRadians(-45));

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        robot.lift.initializeHardware(hardwareMap);
        robot.lift.initialize();
        robot.arm.initializeHardware(hardwareMap);
        robot.arm.initialize();
        robot.extendo.initializeHardware(hardwareMap);
        robot.extendo.initialize();
        robot.brush.initializeHardware(hardwareMap);
        robot.brush.initialize();


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
                .setLinearHeadingInterpolation(score3Pose.getHeading(), submersible1Pose.getHeading())
                .build();

        PathChain scoreSubmersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
                .setLinearHeadingInterpolation(submersible1Pose.getHeading(), scoreSubmersible1Pose.getHeading())
                .build();

        PathChain submersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSubmersible1Pose), new Point(submersible2ControlPoint), new Point(submersible2Pose)))
                .setLinearHeadingInterpolation(scoreSubmersible1Pose.getHeading(), submersible2Pose.getHeading())
                .build();

        PathChain scoreSubmersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible2Pose), new Point(scoreSubmersible2ControlPoint), new Point(scoreSubmbersible2Pose)))
                .setLinearHeadingInterpolation(submersible2Pose.getHeading(), scoreSubmbersible2Pose.getHeading())
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, scorePreload, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, grab1, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, score1, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, grab2, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, score2, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, grab3, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, score3, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, submersible1, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, scoreSubmersible1, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, submersible2, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, scoreSubmersible2, true, 1)







                )
        );


    }

    @Override
    public void run(){
        follower.update();
        CommandScheduler.getInstance().run();


        //robot.loop();
        robot.lift.loop();
        robot.arm.loop();
        robot.extendo.loopAuto();
        robot.brush.loopAutoBasket();


    }


}