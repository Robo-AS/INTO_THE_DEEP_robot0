package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.util.Robot;


@Config
@Autonomous(name = "SpecimenAutoChassis")
public class SpecimenAutoChassis extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;



    public static Pose startPose = new Pose(7, 64, Math.toRadians(180));
    public static Pose preloadPose = new Pose(38, 68, Math.toRadians(180));

    public static Pose grab1Pose = new Pose(64, 23, Math.toRadians(180));
    public static Pose grab1ControlPoint_1 = new Pose(30, 14, Math.toRadians(180));
    public static Pose grab1ControlPoint_2 = new Pose(54, 46, Math.toRadians(180));

    public static Pose bring1Pose = new Pose(19, 24, Math.toRadians(180));



    public static Pose grab2Pose = new Pose(64, 13, Math.toRadians(180));
    public static Pose grab2ControlPoint = new Pose(68, 24, Math.toRadians(180));

    public static Pose bring2Pose = new Pose(18, 13, Math.toRadians(180));



    public static Pose grab3Pose = new Pose(55, 7, Math.toRadians(180));
    public static Pose grab3ControlPoint = new Pose(57, 13, Math.toRadians(180));

    public static Pose bring3Pose = new Pose(18, 7, Math.toRadians(180));




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
                .setConstantHeadingInterpolation(preloadPose.getHeading())
                .build();


//        PathChain scorePreload = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(startPose), new Point(preloadPoseControlPoint), new Point(preloadPose)))
//                .setConstantHeadingInterpolation(preloadPose.getHeading())
//                .build();
        PathChain grab_bringall = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(grab1ControlPoint_1), new Point(grab1ControlPoint_2), new Point(grab1Pose)))
                .setConstantHeadingInterpolation(grab1Pose.getHeading())
                .addPath(new BezierLine(new Point(grab1Pose), new Point(bring1Pose)))
                .setConstantHeadingInterpolation(bring1Pose.getHeading())
                .addPath(new BezierCurve(new Point(bring1Pose), new Point(grab2ControlPoint), new Point(grab2Pose)))
                .setConstantHeadingInterpolation(grab2Pose.getHeading())
                .addPath(new BezierLine(new Point(grab2Pose), new Point(bring2Pose)))
                .setConstantHeadingInterpolation(bring2Pose.getHeading())
                .addPath(new BezierCurve(new Point(bring2Pose), new Point(grab3ControlPoint), new Point(grab3Pose)))
                .setConstantHeadingInterpolation(grab3Pose.getHeading())
                .addPath(new BezierLine(new Point(grab3Pose), new Point(bring3Pose)))
                .setConstantHeadingInterpolation(bring3Pose.getHeading())
                .build();



        PathChain grab1bring1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(grab1ControlPoint_1), new Point(grab1ControlPoint_2), new Point(grab1Pose)))
                .setConstantHeadingInterpolation(grab1Pose.getHeading())
                .addPath(new BezierLine(new Point(grab1Pose), new Point(bring1Pose)))
                .setConstantHeadingInterpolation(bring1Pose.getHeading())
                .build();


        PathChain grab2bring2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring1Pose), new Point(grab2ControlPoint), new Point(grab2Pose)))
                .setConstantHeadingInterpolation(grab2Pose.getHeading())
                .addPath(new BezierLine(new Point(grab2Pose), new Point(bring2Pose)))
                .setConstantHeadingInterpolation(bring2Pose.getHeading())
                .build();

        PathChain grab3bring3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring2Pose), new Point(grab3ControlPoint), new Point(grab3Pose)))
                .setConstantHeadingInterpolation(grab3Pose.getHeading())
                .addPath(new BezierLine(new Point(grab3Pose), new Point(bring3Pose)))
                .setConstantHeadingInterpolation(bring3Pose.getHeading())
                .build();


        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(grab1ControlPoint_1), new Point(grab1ControlPoint_2), new Point(grab1Pose)))
                .setConstantHeadingInterpolation(grab1Pose.getHeading())
                .build();


        PathChain bring1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1Pose), new Point(bring1Pose)))
                .setConstantHeadingInterpolation(bring1Pose.getHeading())
                .build();


        PathChain grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring1Pose), new Point(grab2ControlPoint), new Point(grab2Pose)))
                .setConstantHeadingInterpolation(grab2Pose.getHeading())
                .build();


        PathChain bring2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab2Pose), new Point(bring2Pose)))
                .setConstantHeadingInterpolation(bring2Pose.getHeading())
                .build();


        PathChain grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring2Pose), new Point(grab3ControlPoint), new Point(grab3Pose)))
                .setConstantHeadingInterpolation(grab3Pose.getHeading())
                .build();

        PathChain bring3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab3Pose), new Point(bring3Pose)))
                .setConstantHeadingInterpolation(bring3Pose.getHeading())
                .build();




        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, scorePreload, true, 1),
                        new WaitCommand(500),
//                        new FollowPath(follower, grab1bring1, true, 1),
//                        new FollowPath(follower, grab2bring2, true, 1),
//                        new FollowPath(follower, grab3bring3, true, 1)
                        new FollowPath(follower, grab1, true, 1),
                        // WaitCommand(500),
                        new FollowPath(follower, bring1, true, 1),
                        //new WaitCommand(500),
                        new FollowPath(follower, grab2, true, 1),
                        //new WaitCommand(500),
                        new FollowPath(follower, bring2, true, 1),
                        //new WaitCommand(500),
                        new FollowPath(follower, grab3, true, 1),
                        //new WaitCommand(500),
                        new FollowPath(follower, bring3, true, 1)



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
        robot.brush.loopAuto();


        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();


    }
}
