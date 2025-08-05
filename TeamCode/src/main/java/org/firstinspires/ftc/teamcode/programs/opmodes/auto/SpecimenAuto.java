package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.SpecimenAuto.IntakeRetractFailSafeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.SpecimenAuto.IntakeRetractSPECIMENAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.PutSpecimenCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;

@Autonomous(name = "SpecimenAuto👽🦄🐲🦖")
public class SpecimenAuto extends LinearOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final ElapsedTime time = new ElapsedTime();
    private final SpecimenPaths specimenPaths = new SpecimenPaths();
    private final int sensorTimeOut = 1500;
    private final int spacimentIntakingTimeOut = 1500;


    public static Pose startPose = new Pose(7.480519480519481, 64.98701298701299, Math.toRadians(180));
    public static Pose preloadPose = new Pose(35.40909090909091, 66.83246753246754, Math.toRadians(180)); //1
    public static Pose preloadSMALLPose = new Pose(39.50909090909091, 66.83246753246754, Math.toRadians(180));

    public static Pose grab1Pose = new Pose(30.623376623376622, 28.753246753246753, Math.toRadians(-22)); //2
    public static Pose grab1ControlPoint_1 = new Pose(28.51948051948052, 71.53246753246754, Math.toRadians(-22));
    public static Pose bring1Pose = new Pose(30.623376623376622, 26.415584415584412, Math.toRadians(-155)); //3

    public static Pose grab2Pose = new Pose(30.623376623376622, 23.84415584415585, Math.toRadians(-36)); //4
    public static Pose bring2Pose = new Pose(30.623376623376622, 21.27272727272727, Math.toRadians(-155)); //5

    public static Pose grab3Pose = new Pose(35.2987012987013, 19.4025974025974, Math.toRadians(-63)); //6
    public static Pose bring3Pose = new Pose(26.883116883116884, 23.14285714285714, Math.toRadians(-150)); //7

    public static Pose takeSpecimenPose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130)); //8

    public static Pose score1Pose = new Pose(37.40909090909091, 60.77922077922078, Math.toRadians(180)); //9
    public static Pose score1SMALLPose = new Pose(39.80909090909091, 60.77922077922078, Math.toRadians(180));

//    public static Pose takeSpecimen2Pose = new Pose(23.376623376623378, 35.53246753246753, Math.toRadians(-130)); //10

    public static Pose score2Pose = new Pose(37.40909090909091, 62.77922077922078, Math.toRadians(180)); //11
    public static Pose score2SMALLPose = new Pose(39.80909090909091, 62.77922077922078, Math.toRadians(180));

//    public static Pose takeSpecimen3Pose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130)); //12

    public static Pose score3Pose = new Pose(37.40909090909091, 64.37922077922078, Math.toRadians(180)); //13
    public static Pose score3SMALLPose = new Pose(39.80909090909091, 64.37922077922078, Math.toRadians(180));

//    public static Pose takeSpecimen4Pose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130)); //14

    public static Pose score4Pose = new Pose(37.40909090909091, 65.17922077922078, Math.toRadians(180)); //13
    public static Pose score4SMALLPose = new Pose(39.80909090909091, 65.17922077922078, Math.toRadians(180));

    public static Pose parkPose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130));

//    public static Pose takeSpecimen5Pose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130)); //14

//    public static Pose score5Pose = new Pose(36.90909090909091, 68.77922077922078, Math.toRadians(180)); //13
//    public static Pose score5SMALLPose = new Pose(39.40909090909091, 68.77922077922078, Math.toRadians(180));



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
//        robot.limelightCamera.initializeHardware(hardwareMap);
//        robot.limelightCamera.initializeBLUE();


        robot.extendo.resetEncoders();
        robot.lift.resetEncoders();




        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain scorePreloadSMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(preloadSMALLPose)))
                .setConstantHeadingInterpolation(preloadSMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();



        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadSMALLPose), new Point(grab1ControlPoint_1), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(preloadSMALLPose.getHeading(), grab1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();


        PathChain bring1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1Pose), new Point(bring1Pose)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), bring1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();


        PathChain grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bring1Pose), new Point(grab2Pose)))
                .setLinearHeadingInterpolation(bring1Pose.getHeading(), grab2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        PathChain bring2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab2Pose), new Point(bring2Pose)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), bring2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        PathChain grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bring2Pose), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(bring2Pose.getHeading(), grab3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(12)
                .build();

        PathChain bring3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab3Pose), new Point(bring3Pose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), bring3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(12)
                .build();

        PathChain take1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bring3Pose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(bring3Pose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();


        PathChain score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();


        PathChain score1SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(score1SMALLPose)))
                .setConstantHeadingInterpolation(score1SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        PathChain take2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1SMALLPose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(score1SMALLPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain score2SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(score2SMALLPose)))
                .setConstantHeadingInterpolation(score2SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        PathChain take3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2SMALLPose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(score2SMALLPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain score3SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(score3SMALLPose)))
                .setConstantHeadingInterpolation(score3SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        PathChain take4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3SMALLPose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(score3SMALLPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain score4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score4Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        PathChain score4SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose), new Point(score4SMALLPose)))
                .setConstantHeadingInterpolation(score4SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4SMALLPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(score4SMALLPose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

//        PathChain take5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score4SMALLPose), new Point(takeSpecimen2Pose)))
//                .setLinearHeadingInterpolation(score4SMALLPose.getHeading(), takeSpecimen2Pose.getHeading())
//                .setZeroPowerAccelerationMultiplier(14)
//                .build();
//
//        PathChain score5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(takeSpecimen2Pose), new Point(score5Pose)))
//                .setLinearHeadingInterpolation(takeSpecimen2Pose.getHeading(), score5Pose.getHeading())
//                .setZeroPowerAccelerationMultiplier(14)
//                .build();
//
//
//        PathChain score5SMALL = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score5Pose), new Point(score5SMALLPose)))
//                .setConstantHeadingInterpolation(score5SMALLPose.getHeading())
//                .setZeroPowerAccelerationMultiplier(10)
//                .build();










        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SetClawStateCommand(Arm.ClawState.OPEN),//don't ask
                        new FollowPath(follower, scorePreload, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetClawStateCommand(Arm.ClawState.CLOSED),
                                                new WaitCommand(200),//300
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),//HERE
                                                new OuttakeGoHighRungCommand()
                                        )
                                ),

                        new FollowPath(follower, scorePreloadSMALL, true, 1),

                        new PutSpecimenCommand(),
                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),//HERE


                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new OuttakeGoBackToIdleFromHighRungCommand(),
                                                new WaitCommand(500),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM),
                                                new WaitUntilCommand(robot.extendo::canPutIntakeDown_AUTO_SPECIMENS),
                                                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
                                        )
                                ),
                        new WaitCommand(150).interruptOn(robot.intake::isSampleDigital), //stabilize pedro
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_1).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACT_TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_1).interruptOn(robot.intake::isSampleDigital),
                                        new WaitCommand(300).interruptOn(robot.intake::isSampleDigital),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.GO_AGAIN_SAMPLE_SPECIMEN_AUTO_GRAB_1).interruptOn(robot.intake::isSampleDigital),
                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP)

                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP)
                                ),
                                () -> robot.intake.isNOTSampleDigital()
                        ),




                        new FollowPath(follower, bring1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_HUMAN_PLAYER)
                                        )

                                ),
                        new WaitCommand(200),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM),



                        new FollowPath(follower, grab2, true, 1),
                        new WaitCommand(150),  //stabilize pedro
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_2),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),




                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACT_TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_2).interruptOn(robot.intake::isSampleDigital),
                                        new WaitCommand(300).interruptOn(robot.intake::isSampleDigital),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.GO_AGAIN_SAMPLE_SPECIMEN_AUTO_GRAB_2).interruptOn(robot.intake::isSampleDigital),
                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP)
                                                .alongWith(
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM)
                                                )

                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP)
                                                .alongWith(
                                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM)
                                                )
                                ),
                                () -> robot.intake.isNOTSampleDigital()
                        ),




                        new FollowPath(follower, bring2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(450),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_HUMAN_PLAYER),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER)



                                        )
                                ),
                        new WaitCommand(200),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM),

                        new FollowPath(follower, grab3, true, 1),
                        new WaitCommand(150),   //stabilize pedro
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_3),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.RETRACT_TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_3).interruptOn(robot.intake::isSampleDigital),
                                        new WaitCommand(300).interruptOn(robot.intake::isSampleDigital),
                                        new SetExtendoStateCommand(Extendo.ExtendoState.GO_AGAIN_SAMPLE_SPECIMEN_AUTO_GRAB_3).interruptOn(robot.intake::isSampleDigital),
                                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP)

                                ),
                                new SequentialCommandGroup(
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                        new SetIntakeAngleCommand(Intake.IntakeAngle.UP)
                                ),
                                () -> robot.intake.isNOTSampleDigital()
                        ),


                        new FollowPath(follower, bring3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN_AUTO_GRAB_2),
                                                new WaitCommand(500),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER)
                                        )
                                ),
                        new WaitCommand(200),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),


                        new FollowPath(follower, take1, true, 1)
                                .alongWith(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO)
                                ),
                        new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitCommand(500), //WAIT FOR THE HUMAN PLAYER TO PUT THE SPECIMEN
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),




                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),

////////////////////////////////////////////////////////SCORE1////////////////////////////////////////////////////////////////////

                        new FollowPath(follower, score1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                                new SetClawStateCommand(Arm.ClawState.OPEN),
                                                                new InstantCommand(specimenPaths::setScore1SmallCompleted),
                                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                                new WaitCommand(500),
                                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                        ),
                                                        new SequentialCommandGroup(
                                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                                new WaitCommand(50),
                                                                new OuttakeGoHighRungCommand()
                                                        ),
                                                        () -> robot.intake.isSpecimenBlockedInRollers()
                                                )

                                        )
                                ),


//                        new WaitCommand(250),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score1SMALL, true, 1)
                                                .alongWith(
                                                        new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER)
                                                ),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                ),
                                new DoesNothingCommand(),
                                () -> !specimenPaths.getScore1SmallCompleted()
                        ),




                        new FollowPath(follower, take2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),



////////////////////////////////////////////////////////SCORE2////////////////////////////////////////////////////////////////////

                        new FollowPath(follower, score2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                                new SetClawStateCommand(Arm.ClawState.OPEN),
                                                                new InstantCommand(specimenPaths::setScore2SmallCompleted),
                                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                                new WaitCommand(500),
                                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                        ),
                                                        new SequentialCommandGroup(
                                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                                new WaitCommand(50),
                                                                new OuttakeGoHighRungCommand()
                                                        ),
                                                        () -> robot.intake.isSpecimenBlockedInRollers()
                                                )

                                        )
                                ),
//                        new WaitCommand(250),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score2SMALL, true, 1)
                                                .alongWith(
                                                        new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER)
                                                ),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                ),
                                new DoesNothingCommand(),
                                () -> !specimenPaths.getScore2SmallCompleted()
                        ),



                        new FollowPath(follower, take3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),

////////////////////////////////////////////////////////SCORE3////////////////////////////////////////////////////////////////////
                        new FollowPath(follower, score3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                                new SetClawStateCommand(Arm.ClawState.OPEN),
                                                                new InstantCommand(specimenPaths::setScore3SmallCompleted),
                                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                                new WaitCommand(500),
                                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                        ),
                                                        new SequentialCommandGroup(
                                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                                new WaitCommand(50),
                                                                new OuttakeGoHighRungCommand()
                                                        ),
                                                        () -> robot.intake.isSpecimenBlockedInRollers()
                                                )

                                        )
                                ),
//                        new WaitCommand(250),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score3SMALL, true, 1)
                                                .alongWith(
                                                        new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER)
                                                ),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                ),
                                new DoesNothingCommand(),
                                () -> !specimenPaths.getScore3SmallCompleted()
                        ),




                        new FollowPath(follower, take4, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.extendo::canPutIntakeDown),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new IntakeRetractFailSafeCommand(),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        new SequentialCommandGroup(
                                                new IntakeRetractFailSafeCommand(),
                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                new WaitCommand(300),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                                                new WaitCommand(1000),
                                                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(spacimentIntakingTimeOut),
                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                        ),
                                        robot.intake::isSpecimenBlocked
                                ),
                                new DoesNothingCommand(),
                                robot.intake::isNOTSampleDigital
                        ),



////////////////////////////////////////////////////////SCORE4////////////////////////////////////////////////////////////////////

                        new FollowPath(follower, score4, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new ConditionalCommand(
                                                        new SequentialCommandGroup(
                                                                new SetClawStateCommand(Arm.ClawState.OPEN),
                                                                new InstantCommand(specimenPaths::setScore4SmallCompleted),
                                                                new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                                new WaitCommand(500),
                                                                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                                        ),
                                                        new SequentialCommandGroup(
                                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                                new WaitCommand(50),
                                                                new OuttakeGoHighRungCommand()
                                                        ),
                                                        () -> robot.intake.isSpecimenBlockedInRollers()
                                                )
                                        )
                                ),
//                        new WaitCommand(250),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPath(follower, score4SMALL, true, 1)
                                                .alongWith(
                                                        new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER)
                                                ),
                                        new SetIntakeStateCommand(Intake.IntakeState.IDLE)
                                ),
                                new DoesNothingCommand(),
                                () -> !specimenPaths.getScore4SmallCompleted()
                        ),


                        new FollowPath(follower, park, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                )

                )
        );















        waitForStart();

        while(opModeIsActive()){
            follower.update();
            CommandScheduler.getInstance().run();

            robot.lift.loop();
            robot.arm.loopAuto();
            robot.extendo.loopAuto();
            robot.intake.loopAuto();


            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

        }

    }
}
