package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

@Config
@Autonomous(name = "FUCKING_TEST")
public class FUCKING_TEST extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    public static double zpam = 16;


    public static Pose startPose = new Pose(16, 114, Math.toRadians(0));
    public static Pose endPose = new Pose(61, 114, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);


        PathChain go_forward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zpam)
                .build();

        PathChain go_back = follower.pathBuilder()
                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zpam)
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, go_forward, true, 1),
                        //new WaitCommand(1000),
                        new FollowPath(follower, go_back, true, 1),
                        //new WaitCommand(1000),
                        new FollowPath(follower, go_forward, true, 1),
                       // new WaitCommand(1000),
                        new FollowPath(follower, go_back, true, 1)
                )

        );


        waitForStart();

        while(opModeIsActive()){
            follower.update();
            CommandScheduler.getInstance().run();
        }

    }
}
