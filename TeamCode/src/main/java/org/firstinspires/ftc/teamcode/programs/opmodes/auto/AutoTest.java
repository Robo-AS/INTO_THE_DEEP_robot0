package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoTest", group = "Tests")
public class AutoTest extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public static double scorePose1_X = 14, scorePose1_Y = 127, scorePose1_HEADING = -45;
    public static double pickupSample1_X = 27, pickupSample1_Y = 124;

    private final Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    private final Pose scorePose1 = new Pose(scorePose1_X, scorePose1_Y, Math.toRadians(scorePose1_HEADING));

    private final Pose pickupSample1 = new Pose(pickupSample1_X, pickupSample1_Y);


    private Path scorePreload;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());
    }


    public void autonomouspathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(-1);
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void loop(){
        follower.update();
        autonomouspathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
