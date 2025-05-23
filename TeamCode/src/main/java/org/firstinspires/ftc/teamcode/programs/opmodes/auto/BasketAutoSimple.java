package org.firstinspires.ftc.teamcode.programs.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeRetractAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.OuttakeGoHighBasketAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.ScoreSampleAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

@Config
@Autonomous(name = "BasketAutoSimple")
public class BasketAutoSimple extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private final BasketPaths basketPaths = new BasketPaths();
    private Follower follower;
    private double loopTime = 0;

    private final ElapsedTime time = new ElapsedTime();


    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));

    public static Pose grab1Pose = new Pose(28, 120.8, Math.toRadians(0));
    public static Pose score1Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose grab2Pose = new Pose(28, 130, Math.toRadians(0));
    public static Pose score2Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose grab3Pose = new Pose(28, 134, Math.toRadians(14));
    public static Pose score3Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

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
        robot.hang.initializeHardware(hardwareMap);
        robot.hang.initialize();

        basketPaths.resetTrajectoryes();




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


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SetClawStateCommand(Arm.ClawState.OPEN), //don't ask
                        new FollowPath(follower, scorePreload, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetClawStateCommand(Arm.ClawState.CLOSED),
                                                new WaitCommand(300),
                                                new OuttakeGoHighBasketAutoCommand(),
                                                new ScoreSampleAutoCommand()
                                        )

                                ),
                                //.andThen(new ScoreSampleAutoCommand()),


                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),//500
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )

                                ),

                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                        new WaitUntilCommand(robot.brush::isSample).withTimeout(1000),
                        new SetBrushStateCommand(Brush.BrushState.IDLE),


                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new InstantCommand(basketPaths::setScore1Completed),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),




                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score1, true, 1)
                                                .alongWith(new IntakeRetractAutoCommand()),
                                        new OuttakeGoHighBasketAutoCommand(),
                                        new ScoreSampleAutoCommand()
                                ),
                                new SequentialCommandGroup(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                        new WaitCommand(300)
                                ),
                                () -> !basketPaths.getscore1()
                        ),





                        new FollowPath(follower, grab2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )

                                ),

                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                        new WaitUntilCommand(robot.brush::isSample).withTimeout(1000),
                        new SetBrushStateCommand(Brush.BrushState.IDLE),

                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new InstantCommand(basketPaths::setScore2Completed),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),




                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score2, true, 1)
                                                .alongWith(new IntakeRetractAutoCommand()),
                                        new OuttakeGoHighBasketAutoCommand(),
                                        new ScoreSampleAutoCommand()
                                ),
                                new SequentialCommandGroup(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                        new WaitCommand(300)
                                ),
                                () -> !basketPaths.getscore2()
                        ),



                        new FollowPath(follower, grab3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )

                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO_NEAR_WALL),
                        new WaitUntilCommand(robot.brush::isSample).withTimeout(2000),
                        new SetBrushStateCommand(Brush.BrushState.IDLE),

                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new InstantCommand(basketPaths::setScore3Completed),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),




                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score3, true, 1)
                                                .alongWith(new IntakeRetractAutoCommand()),
                                        new OuttakeGoHighBasketAutoCommand(),
                                        new ScoreSampleAutoCommand()
                                ),
                                new SequentialCommandGroup(
                                        new SetBrushAngleCommand(Brush.BrushAngle.UP),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
                                ),
                                () -> !basketPaths.getscore3()
                        )


                )
        );


    }

    @Override
    public void run(){
        follower.update();
        CommandScheduler.getInstance().run();


        //robot.loopAuto();
        robot.lift.loop();
        robot.arm.loopAuto();
        robot.extendo.loopAuto();
        robot.brush.loopAutoBasket();


        //telemetry.addData("GRAB TIMER", grabTime);
        telemetry.addData("Score_1", basketPaths.getscore1());
        telemetry.addData("Score_2", basketPaths.getscore2());
        telemetry.addData("Score_3", basketPaths.getscore3());

        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();


    }



}
