package org.firstinspires.ftc.teamcode.programs.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeRetractAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.OuttakeGoHighBasketAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.ScoreSampleAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighBasketCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;



@Config
@Autonomous(name = "NEW_BasketAutoSubmersibleBLUE_CU_UNGHI_BRAT💪🟦")
public class NEW_BasketAutoSubmersibleBLUE_CU_UNGHI_BRAT extends LinearOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    private final BasketPaths basketPaths = BasketPaths.getInstance();
    private Follower follower;
    private double loopTime = 0;

    private final ElapsedTime time = new ElapsedTime();

    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));

    public static Pose grab1Pose = new Pose(21.038961038961038, 128.57142857142858, Math.toRadians(-17));
    public static Pose score1Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));

    public static Pose grab2Pose = new Pose(21.506493506493506, 131.37662337662337, Math.toRadians(0));
    public static Pose score2Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));

    public static Pose grab3Pose = new Pose(21.038961038961038, 133.71428571428572, Math.toRadians(15));
    public static Pose score3Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));

    public static Pose submersible1Pose = new Pose(60.311688311688314, 94.77922077922078, Math.toRadians(270));
    public static Pose submersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(270));

    public static Pose scoreSubmersible1Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));
    public static Pose scoreSubmersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(-15));


    public static Pose submersible2Pose = new Pose(64.58260979551126, 94.98909806006719, Math.toRadians(275));
    public static Pose submersible2ControlPoint = new Pose(68.79451913000112, 124.34442310571397, Math.toRadians(275));

    public static Pose scoreSubmbersible2Pose = new Pose(14.5, 130, Math.toRadians(-15));
    public static Pose scoreSubmersible2ControlPoint = new Pose(68.79451913000112, 124.34442310571397, Math.toRadians(-15));

    public static Pose submersible3Pose = new Pose(64.58260979551126, 94.98909806006719, Math.toRadians(265));
    public static Pose submersible3ControlPoint = new Pose(68.79451913000112, 124.34442310571397, Math.toRadians(265));

    public static Pose scoreSubmbersible3Pose = new Pose(15, 129.5, Math.toRadians(-15));
    public static Pose scoreSubmersible3ControlPoint = new Pose(68.79451913000112, 124.34442310571397, Math.toRadians(-15));



    @Override
    public void runOpMode() throws InterruptedException {
        robot.intake.desiredSampleColor = Intake.DesiredSampleColor.BOTH;
        CommandScheduler.getInstance().reset();
        Globals.RUNNED_AUTO_BASKET = true;
        Globals.TELEOP = false;

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robot.lift.initializeHardware(hardwareMap);
        robot.lift.initialize();
        robot.arm.initializeHardware(hardwareMap);
        robot.arm.initialize();
        robot.extendo.initializeHardware(hardwareMap);
        robot.extendo.initialize();
        robot.intake.initializeHardware(hardwareMap);
        robot.intake.initialize();
        robot.hang.initializeHardware(hardwareMap);
        robot.hang.initialize();

        basketPaths.resetTrajectoryes();


        //AICI RESETEZ ENCODERELE CA NU LE MAI RESETEZ IN INIT_UL LA HARWARE CA SA NU LE MAI DAU RESET SI IN TELEOP
        robot.extendo.resetEncoders();
        robot.lift.resetEncoders();





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

//        PathChain submersible1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(score3Pose), new Point(submersible1ControlPoint), new Point(submersible1Pose)))
//                .setLinearHeadingInterpolation(score3Pose.getHeading(), submersible1Pose.getHeading())
//                .build();
        PathChain submersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3Pose), new Point(submersible1ControlPoint), new Point(submersible1Pose)))
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .setZeroPowerAccelerationMultiplier(16)
                .build();

//        PathChain scoreSubmersible1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
//                .setLinearHeadingInterpolation(submersible1Pose.getHeading(), scoreSubmersible1Pose.getHeading())
//                .build();
        PathChain scoreSubmersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        PathChain submersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSubmersible1Pose), new Point(submersible2ControlPoint), new Point(submersible2Pose)))
                .setLinearHeadingInterpolation(scoreSubmersible1Pose.getHeading(), submersible2Pose.getHeading())
                .build();

        PathChain scoreSubmersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible2Pose), new Point(scoreSubmersible2ControlPoint), new Point(scoreSubmbersible2Pose)))
                .setLinearHeadingInterpolation(submersible2Pose.getHeading(), scoreSubmbersible2Pose.getHeading())
                .build();

        PathChain submersible3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSubmbersible2Pose), new Point(submersible3ControlPoint), new Point(submersible3Pose)))
                .setLinearHeadingInterpolation(scoreSubmbersible2Pose.getHeading(), submersible3Pose.getHeading())
                .build();


        PathChain scoreSubmersible3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible3Pose), new Point(scoreSubmersible3ControlPoint), new Point(scoreSubmbersible3Pose)))
                .setLinearHeadingInterpolation(submersible3Pose.getHeading(), scoreSubmbersible3Pose.getHeading())
                .build();


//        while(opModeInInit()){
//            follower.update();
//        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SetClawStateCommand(Arm.ClawState.OPEN), //don't ask
                        new FollowPath(follower, scorePreload, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetClawStateCommand(Arm.ClawState.CLOSED),
                                                new WaitCommand(100),
                                                new OuttakeGoHighBasketAutoCommand(),
                                                new ScoreSampleAutoCommand()
                                        )

                                ),
                        new InstantCommand(basketPaths::setScorePreloadCompleted),

                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(300),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                                                new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                                                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
                                        )
                                ),
                        //new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(1000),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new InstantCommand(basketPaths::setScore1Completed),
                                () -> robot.intake.sampleState == Intake.SampleState.IS
                        ),

                        new ConditionalCommand(
                                //do this:
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
                                //else:
                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                //if:
                                () -> !basketPaths.getscore1()
                        ),


                        new FollowPath(follower, grab2, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                                                new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                                                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
                                        )
                                ),
                        //new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(1000),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new InstantCommand(basketPaths::setScore2Completed),
                                () -> robot.intake.sampleState == Intake.SampleState.IS
                        ),

                        new ConditionalCommand(
                                //do this:
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
                                //else:
                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                //if:
                                () -> !basketPaths.getscore2()
                        ),


                        new FollowPath(follower, grab3, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(300),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                                                new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                                                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
                                        )
                                ),
                        //new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(1000),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new ConditionalCommand(
                                new DoesNothingCommand(),
                                new InstantCommand(basketPaths::setScore3Completed),
                                () -> robot.intake.sampleState == Intake.SampleState.IS
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

                                        new ScoreSampleAutoCommand(),
                                        new OuttakeGoBackToIdleFromHighBasketCommand()
                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
                                ),
                                () -> !basketPaths.getscore3()
                        ),
                        new InstantCommand(basketPaths::setScore3Completed),


                        new FollowPath(follower, submersible1, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand()
                                )









                )
        );

        waitForStart();


        while(opModeIsActive()){
            follower.update();
            CommandScheduler.getInstance().run();


            //robot.loop();
            robot.lift.loop();
            robot.arm.loopAuto();
            robot.extendo.loopAuto();

            robot.intake.loopAuto();


            telemetry.addData("SCORE_PRELOAD_COMPLETED", basketPaths.SCORE_PRELOAD_COMPLETED);
            telemetry.addData("Current Position LIFT", robot.lift.liftMotor.getCurrentPosition());
            telemetry.addData("Current Position EXTENDO", robot.extendo.extendoMotor.getCurrentPosition());


            //telemetry.addData("GRAB TIMER", grabTime);
//            telemetry.addData("Score_1", basketPaths.getscore1());
//            telemetry.addData("Score_2", basketPaths.getscore2());
//            telemetry.addData("Score_3", basketPaths.getscore3());


            telemetry.addData("COLOUR", robot.intake.intakedSampleColor);
            telemetry.addData("THERE IS", robot.intake.sampleState);
            telemetry.addData("DESIRED", robot.intake.desiredSampleColor);


            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();


        }


    }
}
