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
@Autonomous(name = "SpecimenAutoChassisTest2")
public class SpecimenAutoChassisTest2 extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;



    public static Pose startPose = new Pose(7, 64, Math.toRadians(180));
    public static Pose preloadPose = new Pose(38, 68, Math.toRadians(180));

    public static Pose grab1Pose = new Pose(57, 26, Math.toRadians(180));
    public static Pose grab1ControlPoint_1 = new Pose(30, 14, Math.toRadians(180));
    public static Pose grab1ControlPoint_2 = new Pose(54, 46, Math.toRadians(180));

    public static Pose bring1Pose = new Pose(50, 23, Math.toRadians(180));
    public static Pose bring1PoseControlPoint = new Pose(-10, 22, Math.toRadians(180));



    public static Pose grab2bring2Pose = new Pose(50, 13, Math.toRadians(180));
    public static Pose grab2ControlPoint1 = new Pose(82, 11, Math.toRadians(180));
    public static Pose grab2ControlPoint2 = new Pose(-20, 14, Math.toRadians(180));
    public static Pose grab2ControlPoint3 = new Pose(23, 15, Math.toRadians(180));


    public static Pose grab3bring3Pose = new Pose(24, 7, Math.toRadians(180));
    public static Pose grab3ControlPoint = new Pose(68, 5, Math.toRadians(180));

    public static Pose take1Pose = new Pose(25, 38, Math.toRadians(225));
    public static Pose take1PoseControlPoint_1 = new Pose(15, 4, Math.toRadians(225));
    public static Pose take1PoseControlPoint_2 = new Pose(39, 31, Math.toRadians(225));




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

        PathChain all = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(grab1ControlPoint_1), new Point(grab1ControlPoint_2), new Point(grab1Pose)))
                .setConstantHeadingInterpolation(grab1Pose.getHeading())
                .addPath(new BezierCurve(new Point(grab1Pose), new Point(bring1PoseControlPoint), new Point(bring1Pose)))
                .setConstantHeadingInterpolation(bring1Pose.getHeading())
                .addPath(new BezierCurve(new Point(bring1Pose), new Point(grab2ControlPoint1), new Point(grab2ControlPoint2), new Point(grab2ControlPoint3), new Point(grab2bring2Pose)))
                .setConstantHeadingInterpolation(grab2bring2Pose.getHeading())
                .addPath(new BezierCurve(new Point(grab2bring2Pose), new Point(grab3ControlPoint), new Point(grab3bring3Pose)))
                .setConstantHeadingInterpolation(grab3bring3Pose.getHeading())
//                .addPath(new BezierCurve(new Point(grab3bring3Pose), new Point(take1PoseControlPoint_1), new Point(take1PoseControlPoint_2), new Point(take1Pose)))
//                .setLinearHeadingInterpolation(grab3bring3Pose.getHeading(), take1Pose.getHeading())
                .build();

//        PathChain scorePreload = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(startPose), new Point(preloadPoseControlPoint), new Point(preloadPose)))
//                .setConstantHeadingInterpolation(preloadPose.getHeading())
//                .build();

        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(grab1ControlPoint_1), new Point(grab1ControlPoint_2), new Point(grab1Pose)))
                .setConstantHeadingInterpolation(grab1Pose.getHeading())
                .build();

        PathChain bring1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab1Pose), new Point(bring1PoseControlPoint), new Point(bring1Pose)))
                .setConstantHeadingInterpolation(bring1Pose.getHeading())
                .build();

        PathChain grab2bring2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring1Pose), new Point(grab2ControlPoint1), new Point(grab2ControlPoint2), new Point(grab2ControlPoint3), new Point(grab2bring2Pose)))
                .setConstantHeadingInterpolation(grab2bring2Pose.getHeading())
                .build();


        PathChain grab3bring3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab2bring2Pose), new Point(grab3ControlPoint), new Point(grab3bring3Pose)))
                .setConstantHeadingInterpolation(grab3bring3Pose.getHeading())
                .build();

        PathChain take1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab3bring3Pose), new Point(take1PoseControlPoint_1), new Point(take1PoseControlPoint_2), new Point(take1Pose)))
                .setLinearHeadingInterpolation(grab3bring3Pose.getHeading(), take1Pose.getHeading())
                .build();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, scorePreload, true, 1),
                        new WaitCommand(500),
//                        new FollowPath(follower, grab1bring1, true, 1),
//                        new FollowPath(follower, grab2bring2, true, 1),
//                        new FollowPath(follower, grab3bring3, true, 1)
                        new FollowPath(follower, all, true, 0.7)
                        // WaitCommand(500),
//                        new FollowPath(follower, bring1, true, 1),
//                        //new WaitCommand(500),
//                        new FollowPath(follower, grab2bring2, true, 1),
//                        //new WaitCommand(500),
//                        new FollowPath(follower, grab3bring3, true, 1)
                        //new WaitCommand(500),





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
