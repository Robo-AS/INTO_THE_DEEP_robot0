package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
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

@Autonomous(name = "SpecimenAuto_PATHS_ONLY👽🦄🐲🦖")
public class SpecimenAuto_PATHS_ONLY extends LinearOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    private Follower follower;
    private double loopTime = 0;
    private final ElapsedTime time = new ElapsedTime();
    private final SpecimenPaths specimenPaths = new SpecimenPaths();
    private final int sensorTimeOut = 700;


    public static Pose startPose = new Pose(7.480519480519481, 64.98701298701299, Math.toRadians(180));
    public static Pose preloadPose = new Pose(34.40909090909091, 71.53246753246754, Math.toRadians(180)); //1
    public static Pose preloadSMALLPose = new Pose(38.40909090909091, 71.53246753246754, Math.toRadians(180));

    public static Pose grab1Pose = new Pose(30.623376623376622, 28.753246753246753, Math.toRadians(-25)); //2
    public static Pose grab1ControlPoint_1 = new Pose(28.51948051948052, 71.53246753246754, Math.toRadians(-25));
    public static Pose bring1Pose = new Pose(30.623376623376622, 26.415584415584412, Math.toRadians(-155)); //3

    public static Pose grab2Pose = new Pose(30.623376623376622, 23.84415584415585, Math.toRadians(-40)); //4
    public static Pose bring2Pose = new Pose(30.623376623376622, 21.27272727272727, Math.toRadians(-155)); //5

    public static Pose grab3Pose = new Pose(39.272727272727266, 19.636363636363633, Math.toRadians(-70)); //6
    public static Pose bring3Pose = new Pose(26.883116883116884, 23.14285714285714, Math.toRadians(-150)); //7

    public static Pose takeSpecimen1Pose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130));




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
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        PathChain scorePreloadSMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(preloadSMALLPose)))
                .setConstantHeadingInterpolation(preloadSMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();



        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadSMALLPose), new Point(grab1ControlPoint_1), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(preloadSMALLPose.getHeading(), grab1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();


        PathChain bring1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1Pose), new Point(bring1Pose)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), bring1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();


        PathChain grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bring1Pose), new Point(grab2Pose)))
                .setLinearHeadingInterpolation(bring1Pose.getHeading(), grab2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        PathChain bring2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab2Pose), new Point(bring2Pose)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), bring2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        PathChain grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bring2Pose), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(bring2Pose.getHeading(), grab3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        PathChain bring3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab3Pose), new Point(bring3Pose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), bring3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();

        PathChain take1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bring3Pose), new Point(takeSpecimen1Pose)))
                .setLinearHeadingInterpolation(bring3Pose.getHeading(), takeSpecimen1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();








        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
//                        new SetClawStateCommand(Arm.ClawState.OPEN),//don't ask
                        new FollowPath(follower, scorePreload, true, 1),

                        new FollowPath(follower, scorePreloadSMALL, true, 1),

                        new FollowPath(follower, grab1, true, 1),
                         new WaitCommand(10000),

                        new FollowPath(follower, bring1, true, 1),
                        new WaitCommand(1000),

                        new FollowPath(follower, grab2, true, 1),
                        new WaitCommand(10000),


                        new FollowPath(follower, bring2, true, 1),
                        new WaitCommand(5000),


                        new FollowPath(follower, grab3, true, 1),
                        new WaitCommand(10000),


                        new FollowPath(follower, bring3, true, 1),
                        new WaitCommand(5000),

                        new FollowPath(follower, take1, true, 1)


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
