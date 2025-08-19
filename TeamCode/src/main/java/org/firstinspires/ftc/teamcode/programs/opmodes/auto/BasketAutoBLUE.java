package org.firstinspires.ftc.teamcode.programs.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto.IntakeRetractBASKETAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto.IntakeRetractBASKETAutoSUBMERSIBLECommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto.OuttakeGoHighBasketAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto.ScoreSampleAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeThrowingCommandAuto;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.LimelightCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighBasketCommand_AUTO;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;


@Config
@Autonomous(name = "BasketAutoBLUE💪🟦🦺")
public class BasketAutoBLUE extends LinearOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    private final BasketPaths basketPaths = BasketPaths.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final int sensorTimeOut = 700;

    private final ElapsedTime time = new ElapsedTime();


    public static Pose startPose = new Pose(7, 112, Math.toRadians(0));
    public static Pose preloadPose = new Pose(13.324675324675326, 129.97402597402598, Math.toRadians(-17));//1

    public static Pose grab1Pose = new Pose(19.16883116883117, 128.57142857142858, Math.toRadians(-17));//2
    public static Pose score1Pose = new Pose(14.961038961038962, 132.54545454545453, Math.toRadians(-7));//3

    public static Pose grab3Pose = new Pose(21.038961038961038, 133.71428571428572, Math.toRadians(17));//6
    public static Pose score3Pose = new Pose(14.025974025974026, 131.14285714285714, Math.toRadians(-35));//7

    public static Pose submersible1Pose_FIRST = new Pose(46.28571428571429, 111.97402597402598, Math.toRadians(-35));//6
    public static Pose submersible1Pose_SECOND = new Pose(60.77922077922078, 94.14285714285712, Math.toRadians(-90));//7
    public static Pose submersible1Pose_SECONDControlPoint = new Pose(62.18181818181818, 102.85714285714286, Math.toRadians(-90));

    public static Pose scoreSubmersible1Pose_FIRST = new Pose(46.28571428571429, 111.97402597402598, Math.toRadians(-35));       //8
    public static Pose scoreSubmersible1Pose_FIRSTControlPoint = new Pose(62.18181818181818, 102.85714285714286, Math.toRadians(-35));
    public static Pose scoreSubmersible1Pose_SECOND = new Pose(14.25974025974026, 131.14285714285714, Math.toRadians(-35));    //9


    public static Pose submersible2Pose_FIRST = new Pose(46.28571428571429, 111.97402597402598, Math.toRadians(-35));
    public static Pose submersible2Pose_SECOND = new Pose(60.77922077922078, 94.27922077922078, Math.toRadians(-90));
    public static Pose submersible2Pose_SECONDControlPoint = new Pose(62.18181818181818, 102.85714285714286, Math.toRadians(-90));

    public static Pose scoreSubmersible2Pose_FIRST = new Pose(46.28571428571429, 111.97402597402598, Math.toRadians(-35));
    public static Pose scoreSubmersible2Pose_FIRSTControlPoint = new Pose(62.18181818181818, 102.85714285714286, Math.toRadians(-35));
    public static Pose scoreSubmersible2Pose_SECOND = new Pose(14.25974025974026, 131.14285714285714, Math.toRadians(-35));


    public static Pose submersible3Pose_FIRST = new Pose(47.922077922077925, 113.61038961038962, Math.toRadians(-35));
    public static Pose submersible3Pose_SECOND = new Pose(63.81818181818182, 94.14285714285712, Math.toRadians(-90));
    public static Pose submersible3Pose_SECONDControlPoint = new Pose(65.92207792207792, 104.02597402597402, Math.toRadians(-90));

    public static Pose scoreSubmersible3Pose_FIRST = new Pose(47.922077922077925, 113.61038961038962, Math.toRadians(-35));
    public static Pose scoreSubmersible3Pose_FIRSTControlPoint = new Pose(65.92207792207792, 104.02597402597402, Math.toRadians(-35));
    public static Pose scoreSubmersible3Pose_SECOND = new Pose(14.25974025974026, 131.14285714285714, Math.toRadians(-35));



    public PathChain changeHeadingSubmersible1;
    public PathChain changeHeadingSubmersible2;
    public PathChain changeHeadingSubmersible3;
    public PathChain initial;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
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
        robot.sweeper.initializeHardware(hardwareMap);
        robot.sweeper.initialize();
        robot.limelightCamera.initializeHardware(hardwareMap);
        robot.limelightCamera.initializeBLUE();


        basketPaths.resetTrajectoryes();


        //AICI RESETEZ ENCODERELE CA NU LE MAI RESETEZ IN INIT_UL LA HARWARE CA SA NU LE MAI DAU RESET SI IN TELEOP
        robot.extendo.resetEncoders();
        robot.lift.resetEncoders();





        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), grab1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1Pose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), score1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), grab3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), score3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain submersible1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(submersible1Pose_FIRST)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), submersible1Pose_FIRST.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .addPath(new BezierCurve(new Point(submersible1Pose_FIRST), new Point(submersible1Pose_SECONDControlPoint), new Point(submersible1Pose_SECOND)))
                .setLinearHeadingInterpolation(submersible1Pose_FIRST.getHeading(), submersible1Pose_SECOND.getHeading())
//                .setConstantHeadingInterpolation(submersible1Pose_SECOND.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();


        PathChain scoreSubmersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible1Pose_SECOND), new Point(scoreSubmersible1Pose_FIRSTControlPoint), new Point(scoreSubmersible1Pose_FIRST)))
                .setLinearHeadingInterpolation(submersible1Pose_SECOND.getHeading(), scoreSubmersible1Pose_FIRST.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .addPath(new BezierLine(new Point(scoreSubmersible1Pose_FIRST), new Point(scoreSubmersible1Pose_SECOND)))
                .setLinearHeadingInterpolation(scoreSubmersible1Pose_FIRST.getHeading(), scoreSubmersible1Pose_SECOND.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain submersible2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSubmersible1Pose_SECOND), new Point(submersible2Pose_FIRST)))
                .setLinearHeadingInterpolation(scoreSubmersible1Pose_SECOND.getHeading(), submersible2Pose_FIRST.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .addPath(new BezierCurve(new Point(submersible2Pose_FIRST), new Point(submersible2Pose_SECONDControlPoint), new Point(submersible2Pose_SECOND)))
                .setLinearHeadingInterpolation(submersible2Pose_FIRST.getHeading(), submersible2Pose_SECOND.getHeading())
//                .setConstantHeadingInterpolation(submersible2Pose_SECOND.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        PathChain scoreSubmersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible2Pose_SECOND), new Point(scoreSubmersible2Pose_FIRSTControlPoint), new Point(scoreSubmersible2Pose_FIRST)))
                .setLinearHeadingInterpolation(submersible2Pose_SECOND.getHeading(), scoreSubmersible2Pose_FIRST.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .addPath(new BezierLine(new Point(scoreSubmersible2Pose_FIRST), new Point(scoreSubmersible2Pose_SECOND)))
                .setLinearHeadingInterpolation(scoreSubmersible2Pose_FIRST.getHeading(), scoreSubmersible2Pose_SECOND.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();


        PathChain submersible3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSubmersible2Pose_SECOND), new Point(submersible3Pose_FIRST)))
                .setLinearHeadingInterpolation(scoreSubmersible2Pose_SECOND.getHeading(), submersible3Pose_FIRST.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .addPath(new BezierCurve(new Point(submersible3Pose_FIRST), new Point(submersible3Pose_SECONDControlPoint), new Point(submersible3Pose_SECOND)))
                .setLinearHeadingInterpolation(submersible3Pose_FIRST.getHeading(), submersible3Pose_SECOND.getHeading())
//                .setConstantHeadingInterpolation(submersible3Pose_SECOND.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();


        PathChain scoreSubmersible3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible3Pose_SECOND), new Point(scoreSubmersible3Pose_FIRSTControlPoint), new Point(scoreSubmersible3Pose_FIRST)))
                .setLinearHeadingInterpolation(submersible3Pose_SECOND.getHeading(), scoreSubmersible3Pose_FIRST.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .addPath(new BezierLine(new Point(scoreSubmersible3Pose_FIRST), new Point(scoreSubmersible3Pose_SECOND)))
                .setLinearHeadingInterpolation(scoreSubmersible3Pose_FIRST.getHeading(), scoreSubmersible3Pose_SECOND.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();







        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SetClawStateCommand(Arm.ClawState.OPEN), //don't ask
                        new FollowPath(follower, scorePreload, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetClawStateCommand(Arm.ClawState.CLOSED),
                                                new WaitCommand(100),
                                                new OuttakeGoHighBasketAutoCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO)
                                        )

                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO_BASKET_GRAB_1),
                        new ScoreSampleAutoCommand(),
                        new InstantCommand(basketPaths::setScorePreloadCompleted),
                        new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),


                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new OuttakeGoBackToIdleFromHighBasketCommand_AUTO(),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        )

                                ),



                        new FollowPath(follower, score1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoCommand(),
                                                new OuttakeGoHighBasketAutoCommand()
                                                        .alongWith(
                                                                new SequentialCommandGroup(
                                                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO_BASKET_GRAB_2_FIRST_MOVE),
                                                                        new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                                                                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN)
                                                                )

                                                        )
                                        )
                                ),
                        new ScoreSampleAutoCommand(),
                        new InstantCommand(basketPaths::setScore1Completed),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO_BASKET_GRAB_2_SECOND_MOVE),
                        new OuttakeGoBackToIdleFromHighBasketCommand_AUTO().alongWith(new SetIntakeStateCommand(Intake.IntakeState.INTAKING)),




                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                        new IntakeRetractBASKETAutoCommand(),
                        new OuttakeGoHighBasketAutoCommand(),
                        new ScoreSampleAutoCommand(),
                        new InstantCommand(basketPaths::setScore2Completed),



                        new FollowPath(follower, grab3, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand_AUTO()
                                                .alongWith(
                                                        new SequentialCommandGroup(
                                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO_BASKET_GRAB_3_FIRST_MOVE),
                                                                new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                                                                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN)
                                                        )
                                                )

                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_AUTO_BASKET_GRAB_3_SECOND_MOVE),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),


                        new FollowPath(follower, score3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoCommand(),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),

                        new ScoreSampleAutoCommand(),
                        new OuttakeGoBackToIdleFromHighBasketCommand_AUTO(),
                        new InstantCommand(basketPaths::setScore3Completed),



//////////////////////////////////////////////////SUBMERSIBLE 1///////////////////////////////////////////////////

                        new FollowPath(follower, submersible1, true, 1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand_AUTO()
                                ),
                        new WaitCommand(300),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(this::runLimelightHeadingPath_1),
                        new LimelightCommand(),

                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)

                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),

                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitUntilCommand(robot.extendo::retractFinished),
                                                new InstantCommand(this::returnToInitSubmersible1)
                                        ),
                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(this::runLimelightHeadingPath_1),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitUntilCommand(robot.extendo::retractFinished),
                                                new InstantCommand(this::returnToInitSubmersible1)
                                        ),
                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(this::runLimelightHeadingPath_1),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new FollowPath(follower, scoreSubmersible1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoSUBMERSIBLECommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.STABILIZER_AUTO_BASKET),
                                                new WaitCommand(400),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),
                        new ScoreSampleAutoCommand(),


                        /////////////////////////////////SUBMERSIBLE 2/////////////////////////////////////////
                        new FollowPath(follower, submersible2, true,  1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new OuttakeGoBackToIdleFromHighBasketCommand_AUTO(),
                                                new WaitCommand(300),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)

                                        )

                                ),
                        new WaitCommand(300),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(this::runLimelightHeadingPath_2),
                        new LimelightCommand(),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),

                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),

                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitUntilCommand(robot.extendo::retractFinished),
                                                new InstantCommand(this::returnToInitSubmersible2)
                                        ),
                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(this::runLimelightHeadingPath_2),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),

                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitUntilCommand(robot.extendo::retractFinished),
                                                new InstantCommand(this::returnToInitSubmersible2)
                                        ),

                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(this::runLimelightHeadingPath_2),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new FollowPath(follower, scoreSubmersible2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoSUBMERSIBLECommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.STABILIZER_AUTO_BASKET),
                                                new WaitCommand(400),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),
                        new ScoreSampleAutoCommand(),


                        ///////////////////////////////////SUBMERSIBLE 3/////////////////////////////////////////
                        new FollowPath(follower, submersible3, true,  1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new OuttakeGoBackToIdleFromHighBasketCommand_AUTO(),
                                                new WaitCommand(300),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)

                                        )

                                ),
                        new WaitCommand(300),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(this::runLimelightHeadingPath_3),
                        new LimelightCommand(),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),

                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitUntilCommand(robot.extendo::retractFinished),
                                                new InstantCommand(this::returnToInitSubmersible3)
                                        ),

                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(this::runLimelightHeadingPath_3),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new ConditionalCommand(
                                new InstantCommand(()->robot.intake.updateSampleColor()),
                                new DoesNothingCommand(),
                                robot.intake::isSampleDigital
                        ),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
//                                        new InstantCommand(()->robot.intake.updateSampleColor()),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new IntakeThrowingCommandAuto(),
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE_AFTER_THROWING).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.extendo::limelightTakePoseAfterThrowingFinished).interruptOn(robot.intake::isSampleDigital),
                                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                ),
                                                new DoesNothingCommand(),
                                                robot.intake::isColorWrongBasketAutoBLUE
                                        )
                                ),

                                new SequentialCommandGroup(
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitUntilCommand(robot.extendo::retractFinished),
                                                new InstantCommand(this::returnToInitSubmersible3)
                                        ),
                                        new WaitCommand(300),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                                        new InstantCommand(this::runLimelightHeadingPath_3),
                                        new LimelightCommand()
                                ),
                                robot.intake::isSampleDigital
                        ),


                        new FollowPath(follower, scoreSubmersible3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoSUBMERSIBLECommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.STABILIZER_AUTO_BASKET),
                                                new WaitCommand(400),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),
                        new ScoreSampleAutoCommand(),
                        new OuttakeGoBackToIdleFromHighBasketCommand_AUTO(),
                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
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


//            telemetry.addData("SCORE_PRELOAD_COMPLETED", basketPaths.SCORE_PRELOAD_COMPLETED);
//            telemetry.addData("Current Position LIFT", robot.lift.liftMotor.getCurrentPosition());
//            telemetry.addData("Current Position EXTENDO", robot.extendo.extendoMotor.getCurrentPosition());


            //telemetry.addData("GRAB TIMER", grabTime);
//            telemetry.addData("Score_1", basketPaths.getscore1());
//            telemetry.addData("Score_2", basketPaths.getscore2());
//            telemetry.addData("Score_3", basketPaths.getscore3());


            telemetry.addData("COLOUR", robot.intake.intakedSampleColor);
            telemetry.addData("THERE IS", robot.intake.sampleState);
//            telemetry.addData("DESIRED", robot.intake.desiredSampleColor);


            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();


        }

    }



    public void runLimelightHeadingPath_1(){
        changeHeadingSubmersible1 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose_SECOND)))
                .setConstantHeadingInterpolation(robot.limelightCamera.targetAngle + Math.toRadians(-90) - Math.toRadians(0.9))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, changeHeadingSubmersible1, true, 1)
        );
    }

    public void runLimelightHeadingPath_2(){
        changeHeadingSubmersible2 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible2Pose_SECOND)))
                .setConstantHeadingInterpolation(robot.limelightCamera.targetAngle + Math.toRadians(-90) - Math.toRadians(0.9))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, changeHeadingSubmersible2, true, 1)
        );
    }


    public void runLimelightHeadingPath_3(){
        changeHeadingSubmersible3 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible3Pose_SECOND)))
                .setConstantHeadingInterpolation(robot.limelightCamera.targetAngle + Math.toRadians(-90) - Math.toRadians(0.9))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, changeHeadingSubmersible3, true, 1)
        );
    }


    public void returnToInitSubmersible1(){
        initial = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose_SECOND)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, initial, true, 1)
        );
    }

    public void returnToInitSubmersible2(){
        initial = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose_SECOND)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, initial, true, 1)
        );
    }

    public void returnToInitSubmersible3(){
        initial = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose_SECOND)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, initial, true, 1)
        );
    }
}
