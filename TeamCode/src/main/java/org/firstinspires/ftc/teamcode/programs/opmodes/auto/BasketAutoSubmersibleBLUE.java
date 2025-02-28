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
import com.pedropathing.pathgen.BezierCurve;
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
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeRetractSubmersibleAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.OuttakeGoHighBasketAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.ScoreSampleAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighBasketCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

@Config
@Autonomous(name = "BasketAutoSubmersibleBLUE🟦")
public class BasketAutoSubmersibleBLUE extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private final BasketPaths basketPaths = new BasketPaths();
    private Follower follower;
    private double loopTime = 0;

    private final ElapsedTime time = new ElapsedTime();


    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));

    public static Pose grab1Pose = new Pose(26, 120.8, Math.toRadians(0));
    public static Pose score1Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose grab2Pose = new Pose(26, 130, Math.toRadians(0));
    public static Pose score2Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose grab3Pose = new Pose(28, 134, Math.toRadians(14));
    public static Pose score3Pose = new Pose(16.5, 129.5, Math.toRadians(-45));

    public static Pose submersible1Pose = new Pose(60.13670549799417, 94.98909806006719, Math.toRadians(270));
    public static Pose submersible1ControlPoint = new Pose(62.71065009129354, 116.8565842888431, Math.toRadians(270));

    public static Pose scoreSubmersible1Pose = new Pose(16.5, 129.5, Math.toRadians(-45));
    public static Pose scoreSubmersible1ControlPoint = new Pose(62.71065009129354, 117.09057925187031, Math.toRadians(-45));

    public static Pose submersible2Pose = new Pose(64.58260979551126, 94.98909806006719, Math.toRadians(275));
    public static Pose submersible2ControlPoint = new Pose(68.79451913000112, 124.34442310571397, Math.toRadians(275));

    public static Pose scoreSubmbersible2Pose = new Pose(15, 129.5, Math.toRadians(-45));
    public static Pose scoreSubmersible2ControlPoint = new Pose(68.79451913000112, 124.34442310571397, Math.toRadians(-45));


    @Override
    public void initialize() {
        robot.brush.desiredSampleColor = Brush.DesiredSampleColor.BOTH;
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


                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand(),
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
                                new InstantCommand(basketPaths::setScore1Completed),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),




                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score1, true, 1)
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                            new IntakeRetractAutoCommand(),
                                                            new OuttakeGoHighBasketAutoCommand()
                                                        )
                                                ),
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
                                        new OuttakeGoBackToIdleFromHighBasketCommand(),
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
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new IntakeRetractAutoCommand(),
                                                                new OuttakeGoHighBasketAutoCommand()
                                                        )
                                                ),
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
                                        new OuttakeGoBackToIdleFromHighBasketCommand(),
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
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new IntakeRetractAutoCommand(),
                                                                new OuttakeGoHighBasketAutoCommand()
                                                        )
                                                ),

                                        new ScoreSampleAutoCommand()
//                                        new OuttakeGoBackToIdleFromHighBasketCommand()
                                ),
                                new SequentialCommandGroup(
                                        new SetBrushAngleCommand(Brush.BrushAngle.UP),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
                                ),
                                () -> !basketPaths.getscore3()
                        ),

                        new InstantCommand(basketPaths::setScore3Completed), //doar pentru schimbare de loop-uri am asta



                        //DE AICI INCEPE SUBMERSIBLE
                        new FollowPath(follower, submersible1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new OuttakeGoBackToIdleFromHighBasketCommand(),
                                                new WaitCommand(1000),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO)
                                        )

                                ),
                        new SetBrushStateCommand(Brush.BrushState.INTAKING),
                        new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SUBMERSIBLE_1),
                        new WaitUntilCommand(robot.brush::isSample).withTimeout(2000),
                        new SetBrushStateCommand(Brush.BrushState.IDLE),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.THROWING),
                                                new WaitCommand(500),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new DoesNothingCommand(),
                                        () -> robot.brush.intakedSampleColor == Brush.IntakedSampleColor.RED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.THROWING),
                                                new WaitCommand(500),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new DoesNothingCommand(),
                                        () -> robot.brush.intakedSampleColor == Brush.IntakedSampleColor.RED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),





                        new FollowPath(follower, scoreSubmersible1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSubmersibleAutoCommand(),
                                                new WaitCommand(500),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )

                                ),
                        new ScoreSampleAutoCommand(),



                        ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                        new FollowPath(follower, submersible2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new OuttakeGoBackToIdleFromHighBasketCommand(),
                                                new WaitCommand(1000),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO)
                                        )

                                ),
                        new SetBrushStateCommand(Brush.BrushState.INTAKING),
                        new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SUBMERSIBLE_2),
                        new WaitUntilCommand(robot.brush::isSample).withTimeout(2000),
                        new SetBrushStateCommand(Brush.BrushState.IDLE),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.THROWING),
                                                new WaitCommand(500),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new DoesNothingCommand(),
                                        () -> robot.brush.intakedSampleColor == Brush.IntakedSampleColor.RED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.THROWING),
                                                new WaitCommand(500),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new DoesNothingCommand(),
                                        () -> robot.brush.intakedSampleColor == Brush.IntakedSampleColor.RED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.IS
                        ),





                        new FollowPath(follower, scoreSubmersible2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSubmersibleAutoCommand(),
                                                new WaitCommand(500),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )

                                ),
                        new ScoreSampleAutoCommand(),
                        new OuttakeGoBackToIdleFromHighBasketCommand()

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

        if(!basketPaths.getscore3())
            robot.brush.loopAutoBasket();
        else robot.brush.loopAuto();


        //telemetry.addData("GRAB TIMER", grabTime);
        telemetry.addData("Score_1", basketPaths.getscore1());
        telemetry.addData("Score_2", basketPaths.getscore2());
        telemetry.addData("Score_3", basketPaths.getscore3());


        telemetry.addData("COLOUR", robot.brush.intakedSampleColor);
        telemetry.addData("THERE IS", robot.brush.sampleState);
        telemetry.addData("DESIRED", robot.brush.desiredSampleColor);


        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();


    }



}
