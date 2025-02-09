package org.firstinspires.ftc.teamcode.programs.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

@Config
@Autonomous(name = "BasketAuto")
public class BasketAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();

    private Follower follower;

    public static double grab1Pose_X = 28, grab1Pose_Y = 121.5, grab1Pose_HEADING = 0;

    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(14, 127, Math.toRadians(-45));

    public static Pose grab1Pose = new Pose(grab1Pose_X, grab1Pose_Y, Math.toRadians(grab1Pose_HEADING));

    private PathChain scorePreload, grab1;
    //private SequentialCommandGroup placeSample;




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



        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), grab1Pose.getHeading())
                .build();

//        placeSample = new SequentialCommandGroup(
//                new SetLiftStateCommand(Lift.LiftState.HIGH_BASKET),
//                new SetArmStateCommand(Arm.ArmState.HIGH_BASKET)
//        );


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //new SetClawStateCommand(Arm.ClawState.CLOSED),
                        new FollowPath(follower, scorePreload, true, 1),
//                                .alongWith(
//                                        new SetLiftStateCommand(Lift.LiftState.HIGH_BASKET),
//                                        new SetArmStateCommand(Arm.ArmState.HIGH_BASKET)
//                                )
//                                .andThen(new SetClawStateCommand(Arm.ClawState.OPEN))
                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )

                                ),

                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO)

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


    }


}
