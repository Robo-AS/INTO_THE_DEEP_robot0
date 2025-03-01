package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeRetractAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeRetractFailSafeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.PutSpecimenCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.Robot;

@Autonomous(name = "SpecimenAuto_FailSafes_DMNAJUTA🦄")
public class SpecimenAuto_FailSafes_DMNAJUTA extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final ElapsedTime time = new ElapsedTime();
    private final SpecimenPaths specimenPaths = new SpecimenPaths();
    private boolean timeIsUp = false;

    public static Pose startPose = new Pose(7, 64, Math.toRadians(180));
    public static Pose preloadPose = new Pose(34.5, 68, Math.toRadians(180));
    public static Pose preloadSMALLPose = new Pose(38.5, 68, Math.toRadians(180));


    public static Pose grab1Pose = new Pose(63, 22, Math.toRadians(180));
    public static Pose grab1ControlPoint_1 = new Pose(30, 14, Math.toRadians(180));
    public static Pose grab1ControlPoint_2 = new Pose(54, 46, Math.toRadians(180));

    public static Pose bring1Pose = new Pose(40.90909090909091, 22.44155844155845, Math.toRadians(180));
    public static Pose bring1PoseControlPoint = new Pose(39.74025974025975, 25.01298701298701, Math.toRadians(180));


    public static Pose grab2Pose = new Pose(64, 12, Math.toRadians(180));
    public static Pose grab2ControlPoint = new Pose(68, 24, Math.toRadians(180));

    public static Pose bring2Pose = new Pose(41.37662337662338, 11.45454545454546, Math.toRadians(180));
    public static Pose bring2PoseControlPoint = new Pose(40.675324675324674, 14.259740259740258, Math.toRadians(180));


    public static Pose grab3Pose = new Pose(55, 7, Math.toRadians(180));
    public static Pose grab3ControlPoint = new Pose(57, 13, Math.toRadians(180));


    public static Pose bring3Pose = new Pose(22, 25, Math.toRadians(180));
    public static Pose bring3PoseControlPoint1 = new Pose(15, 4, Math.toRadians(180));
    public static Pose bring3PoseControlPoint2 = new Pose(20, 6, Math.toRadians(180));


    public static Pose takeSpecimen1Pose = new Pose(25, 39, Math.toRadians(230));
    public static Pose takeSpecimen1PoseControlPoint = new Pose(23, 29, Math.toRadians(230));

    public static Pose score1Pose = new Pose(34.33766233766234, 59.61038961038961, Math.toRadians(180));//line 9
    public static Pose score1PoseControlPoint = new Pose(28.98701298701299, 51.1948051948052, Math.toRadians(180));

    public static Pose score1SMALLPose = new Pose(38.63766233766234, 59.61038961038961, Math.toRadians(180));


    public static Pose take2Pose = new Pose(25.246753246753247, 39.740259740259745, Math.toRadians(230));//line 10
    public static Pose take2PoseControlPoint = new Pose(28.28571428571428, 50.02597402597402, Math.toRadians(230));


    public static Pose score2Pose = new Pose(34.33766233766234, 61.48051948051948, Math.toRadians(180));//line 11
    public static Pose score2PoseControlPoint = new Pose(25.948051948051948, 50.02597402597402, Math.toRadians(180));

    public static Pose score2SMALLPose = new Pose(38.63766233766234, 61.48051948051948, Math.toRadians(180));

    public static Pose take3Pose = new Pose(25.246753246753247, 39.740259740259745, Math.toRadians(230));//line 12
    public static Pose take3PoseControlPoint = new Pose(25.948051948051948, 50.25974025974024, Math.toRadians(230));


    public static Pose score3Pose = new Pose(34.33766233766234, 63.35064935064937, Math.toRadians(180));//line 13
    public static Pose score3PoseControlPoint = new Pose(27.116883116883116, 54, Math.toRadians(180));

    public static Pose score3SMALLPose = new Pose(38.63766233766234, 63.35064935064937, Math.toRadians(180));


    public static Pose take4Pose = new Pose(25.246753246753247, 39.97402597402599, Math.toRadians(230));//line 14
    public static Pose take4PoseControlPoint = new Pose(27.116883116883116, 54, Math.toRadians(230));


    public static Pose score4Pose = new Pose(34.33766233766234, 65.45454545454547, Math.toRadians(180));//line 15
    public static Pose score4PoseControlPoint = new Pose(27.116883116883116, 54.233766233766254, Math.toRadians(180));

    public static Pose score4SMALLPose = new Pose(38.63766233766234, 65.45454545454547, Math.toRadians(180));

    public static Pose goBackPose = new Pose(34.33766233766234, 65.45454545454547, Math.toRadians(180));




    @Override
    public void runOpMode() throws InterruptedException {
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

        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setConstantHeadingInterpolation(preloadPose.getHeading())
                .build();

        PathChain scorePreloadSMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(preloadSMALLPose)))
                .setConstantHeadingInterpolation(preloadSMALLPose.getHeading())
                .build();




        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(grab1ControlPoint_1), new Point(grab1ControlPoint_2), new Point(grab1Pose)))
                .setConstantHeadingInterpolation(grab1Pose.getHeading())
                .build();


        PathChain bring1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab1Pose), new Point(bring1PoseControlPoint), new Point(bring1Pose)))
                .setConstantHeadingInterpolation(bring1Pose.getHeading())
                .build();


        PathChain grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring1Pose), new Point(grab2ControlPoint), new Point(grab2Pose)))
                .setConstantHeadingInterpolation(grab2Pose.getHeading())
                .build();


        PathChain bring2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab2Pose), new Point(bring2PoseControlPoint), new Point(bring2Pose)))
                .setConstantHeadingInterpolation(bring2Pose.getHeading())
                .build();


        PathChain grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bring2Pose), new Point(grab3ControlPoint), new Point(grab3Pose)))
                .setConstantHeadingInterpolation(grab3Pose.getHeading())
                .build();

        PathChain bring3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab3Pose), new Point(bring3PoseControlPoint1), new Point(bring3PoseControlPoint2), new Point(bring3Pose)))
                .setConstantHeadingInterpolation(bring3Pose.getHeading())
                .addPath(new BezierCurve(new Point(bring3Pose), new Point(takeSpecimen1PoseControlPoint), new Point(takeSpecimen1Pose)))
                .setLinearHeadingInterpolation(bring3Pose.getHeading(), takeSpecimen1Pose.getHeading())
                .build();

        PathChain score1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(takeSpecimen1Pose), new Point(score1PoseControlPoint), new Point(score1Pose)))
                .setLinearHeadingInterpolation(takeSpecimen1Pose.getHeading(), score1Pose.getHeading())
                .build();


        PathChain score1SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(score1SMALLPose)))
                .setConstantHeadingInterpolation(score1SMALLPose.getHeading())
                .build();


        PathChain take2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score1Pose), new Point(take2PoseControlPoint), new Point(take2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), take2Pose.getHeading())
                .build();


        PathChain score2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take2Pose), new Point(score2PoseControlPoint), new Point(score2Pose)))
                .setLinearHeadingInterpolation(take2Pose.getHeading(), score2Pose.getHeading())
                .build();


        PathChain score2SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(score2SMALLPose)))
                .setConstantHeadingInterpolation(score2SMALLPose.getHeading())
                .build();


        PathChain take3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score2Pose), new Point(take3PoseControlPoint), new Point(take3Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), take3Pose.getHeading())
                .build();

        PathChain score3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take3Pose), new Point(score3PoseControlPoint), new Point(score3Pose)))
                .setLinearHeadingInterpolation(take3Pose.getHeading(), score3Pose.getHeading())
                .build();

        PathChain score3SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(score3SMALLPose)))
                .setConstantHeadingInterpolation(score3SMALLPose.getHeading())
                .build();

        PathChain take4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3SMALLPose), new Point(take4PoseControlPoint), new Point(take4Pose)))
                .setLinearHeadingInterpolation(score3SMALLPose.getHeading(), take4Pose.getHeading())
                .build();


        PathChain score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take4Pose), new Point(score4PoseControlPoint), new Point(score4Pose)))
                .setLinearHeadingInterpolation(take4Pose.getHeading(), score4Pose.getHeading())
                .build();

        PathChain score4SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose), new Point(score4SMALLPose)))
                .setConstantHeadingInterpolation(score4SMALLPose.getHeading())
                .build();

        PathChain goBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4SMALLPose), new Point(goBackPose)))
                .setConstantHeadingInterpolation(goBackPose.getHeading())
                .build();


        while (opModeInInit()){
            follower.update();
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //new RunCommand(() -> follower.update()),
                        new InstantCommand(time::reset),
                        new SetClawStateCommand(Arm.ClawState.OPEN),//don't ask
                        new FollowPath(follower, scorePreload, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetClawStateCommand(Arm.ClawState.CLOSED),
                                                new WaitCommand(300),
                                                new OuttakeGoHighRungCommand()
                                        )
                                ),


                        //new WaitCommand(100),
                        new FollowPath(follower, scorePreloadSMALL, true, 1),
                        new WaitCommand(200),
                        new PutSpecimenCommand(),


                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new OuttakeGoBackToIdleFromHighRungCommand(),
                                                new WaitCommand(500),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO)
                                        )
                                )
                                .andThen(new SetBrushAngleCommand(Brush.BrushAngle.DOWN_AUTO)),

                        new FollowPath(follower, bring1, true, 1)
                                .alongWith(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN)
                                ),

                        new FollowPath(follower, grab2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(300),
                                                new SetBrushAngleCommand(Brush.BrushAngle.UP)

                                        )


                                )
                                .andThen(new SetBrushAngleCommand(Brush.BrushAngle.DOWN_AUTO)),


                        new FollowPath(follower, bring2, true, 1)
                                .alongWith(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN)
                                ),

                        new FollowPath(follower, grab3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new WaitCommand(300),
                                                new SetBrushAngleCommand(Brush.BrushAngle.UP)

                                        )
                                ),



                        new FollowPath(follower, bring3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1200),//2000
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN_AUTO),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)


                                        )
                                )
                                .andThen(
                                        new InstantCommand(() -> specimenPaths.setBring3Take1Completed()),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                        new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE)
                                ),



                        //FAIL SAFE FOR TAKE 1
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),






                        new FollowPath(follower, score1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractAutoCommand(),
                                                new OuttakeGoHighRungCommand()
                                        )

                                ),
                        new InstantCommand(() -> specimenPaths.setScore1Completed()),
                        new WaitCommand(250),
                        new FollowPath(follower, score1SMALL, true, 1),
                        new InstantCommand(() -> specimenPaths.setScore1SmallCompleted()),
//                        new WaitCommand(200),





                        new FollowPath(follower, take2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        ),
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN_AUTO),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )



                                )
                                .andThen(
                                        new InstantCommand(() -> specimenPaths.setTake2Completed()),
                                        new WaitCommand(100),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                        new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE)
                                ),


                        //FAIL SAFES FOR TAKE 2
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),


                        new FollowPath(follower, score2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractAutoCommand(),
                                                new OuttakeGoHighRungCommand()
                                        )

                                ),

                        new InstantCommand(() -> specimenPaths.setScore2Completed()),
                        new WaitCommand(250),
                        new FollowPath(follower, score2SMALL, true, 1),
                        new InstantCommand(() -> specimenPaths.setScore2SmallCompleted()),
//                        new WaitCommand(200),




                        new FollowPath(follower, take3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        ),

                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN_AUTO),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )

                                )
                                .andThen(
                                        new InstantCommand(() -> specimenPaths.setTake3Completed()),
                                        new WaitCommand(100),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                        new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE)
                                ),


                        //FAIL SAFES FOR TAKE 3
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),


                        new FollowPath(follower, score3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractAutoCommand(),
                                                new OuttakeGoHighRungCommand()
                                        )

                                ),
                        new InstantCommand(() ->specimenPaths.setScore3Completed()),
                        new WaitCommand(250),
                        new FollowPath(follower, score3SMALL, true, 1),
                        new InstantCommand(() -> specimenPaths.setScore3SmallCompleted()),
//                        new WaitCommand(200),



                        new FollowPath(follower, take4, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        ),
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN_AUTO),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )


                                )
                                .andThen(
                                        new InstantCommand(() -> specimenPaths.setTake4Completed()),
                                        new WaitCommand(100),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                        new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE)
                                ),

                        //FAIL SAFES FOR TAKE 4
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(2000),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.brush::isSample).withTimeout(Globals.TIMEOUT_SPECIMEN_INTAKING),
                                                new SetBrushStateCommand(Brush.BrushState.IDLE)
                                        ),
                                        () -> robot.brush.specimenBlocked == Brush.SpecimenBlocked.BLOCKED
                                ),
                                new DoesNothingCommand(),
                                () -> robot.brush.sampleState == Brush.SampleState.ISNOT
                        ),



                        new FollowPath(follower, score4, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractAutoCommand(),
                                                new OuttakeGoHighRungCommand()
                                        )

                                ),

                        new InstantCommand(() -> specimenPaths.setScore4Completed()),
                        new WaitCommand(250),
                        new FollowPath(follower, score4SMALL, true, 1),
                        new InstantCommand(() -> specimenPaths.setScore4SmallCompleted()),

                        new PutSpecimenCommand(),
//                        new WaitCommand(200),
                        new FollowPath(follower, goBack, true, 1)
                                .alongWith(
                                    new OuttakeGoBackToIdleFromHighRungCommand()
                                )
                )
        );




        while(opModeIsActive()){
            follower.update();
            CommandScheduler.getInstance().run();


            //robot.loop();
            robot.lift.loop();
            robot.arm.loop();
            robot.extendo.loopAuto();
            robot.brush.loopAutoSpecimen();

            if(time.seconds() > 27 && !specimenPaths.allTrajectoriesCompleted()){
                timeIsUp = true;
            }



//            telemetry.addData("X_OFFSET", follower.getXOffset());
//            telemetry.addData("Y_OFFSET", follower.getYOffset());
//            telemetry.addData("HEADING", follower.getHeadingOffset());

//            Pose currentPose = follower.getPose();
//            telemetry.addData("X", currentPose.getX());
//            telemetry.addData("Y", currentPose.getY());
//            telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

            telemetry.addData("Time", time.seconds());

            telemetry.addData("bring3_take1", specimenPaths.getBring3Take1Completed());
            telemetry.addData("score_1", specimenPaths.getScore1Completed());
            telemetry.addData("score_1_SMALL", specimenPaths.getScore1SmallCompleted());
            telemetry.addData("take_2", specimenPaths.getTake2Completed());
            telemetry.addData("score_2", specimenPaths.getScore2Completed());
            telemetry.addData("score_2_SMALL", specimenPaths.getScore2SmallCompleted());
            telemetry.addData("take_3", specimenPaths.getTake3Completed());
            telemetry.addData("score_3", specimenPaths.getScore3Completed());
            telemetry.addData("score_3_SMALL", specimenPaths.getScore3SmallCompleted());
            telemetry.addData("take_4", specimenPaths.getTake4Completed());
            telemetry.addData("score_4", specimenPaths.getScore4Completed());
            telemetry.addData("score_4_SMALL", specimenPaths.getScore4SmallCompleted());


            telemetry.addData("Specimen Bloked?", robot.brush.specimenBlocked);
            if(robot.brush.brushState == Brush.BrushState.INTAKING)
                telemetry.addData("BrushMotorCurrent", robot.brush.brushMotor.getCurrent(CurrentUnit.AMPS));

            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();


            telemetry.addData("X", follower.getXOffset());
            telemetry.addData("Y", follower.getYOffset());


            telemetry.addData("Heading", follower.getHeadingOffset());
        }


    }


    public void cancelAuto(){
        follower.breakFollowing();

        CommandScheduler.getInstance().schedule(
                new ConditionalCommand(
                        new DoesNothingCommand(),
                        new SequentialCommandGroup(
                                new SetBrushStateCommand(Brush.BrushState.IDLE),
                                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
                        ),
                        () -> robot.extendo.extendoState == Extendo.ExtendoState.RETRACTING
                ),

                new ConditionalCommand(
                        new OuttakeGoBackToIdleFromHighRungCommand(),
                        new DoesNothingCommand(),
                        () -> robot.lift.liftState == Lift.LiftState.HIGH_RUNG
                )
        );
    }
}
