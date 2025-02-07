package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Config
public class BasketPaths {

    public static double preloadPose_X = 14, preloadPose_Y = 127, preloadPose_HEADING = -45;

    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(preloadPose_X, preloadPose_Y, Math.toRadians(preloadPose_HEADING));



    public static PathChain preload(){
        return new PathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .build();
    }
}
