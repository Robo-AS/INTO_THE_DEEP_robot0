package org.firstinspires.ftc.teamcode.programs.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto.OuttakeGoHighBasketAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto.ScoreSampleAutoCommand;
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
@Autonomous(name = "TEST_LIMELIGHT_SUBMERSIBLE")
public class TEST_LIMELIGHT_SUBMERSIBLE extends LinearOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    private final BasketPaths basketPaths = BasketPaths.getInstance();
    private Follower follower;
    private double loopTime = 0;

    private final ElapsedTime time = new ElapsedTime();


    public static Pose startPose = new Pose(7, 112, Math.toRadians(-90));
    public static Pose preloadPose = new Pose(13, 127, Math.toRadians(-45));//1

    public static Pose grab1Pose = new Pose(21.038961038961038, 128.57142857142858, Math.toRadians(-17));//2
    public static Pose score1Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));//3

    public static Pose grab2Pose = new Pose(21.506493506493506, 131.37662337662337, Math.toRadians(0));//4
    public static Pose score2Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));//5

    public static Pose grab3Pose = new Pose(21.038961038961038, 133.71428571428572, Math.toRadians(15));//6
    public static Pose score3Pose = new Pose(15.792207792207792, 130.9090909090909, Math.toRadians(-15));//7

    public static Pose submersible1Pose = new Pose(60.311688311688314, 94.27922077922078, Math.toRadians(-90));//8
    public static Pose submersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(-90));

    public static Pose scoreSubmersible1Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));       //9
    public static Pose scoreSubmersible1ControlPoint = new Pose(60.54545454545455, 119.92207792207793, Math.toRadians(-15));


    public static Pose submersible2Pose = new Pose(63.350649350649356, 94.27922077922078, Math.toRadians(-90));             //10
    public static Pose submersible2ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(-90));

    public static Pose scoreSubmbersible2Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));      //11
    public static Pose scoreSubmersible2ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(-15));

    public static Pose submersible3Pose = new Pose(63.350649350649356, 94.27922077922078, Math.toRadians(-90));     //12
    public static Pose submersible3ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(-90));

    public static Pose scoreSubmbersible3Pose = new Pose(15.896103896103895, 130.67532467532467, Math.toRadians(-15));      //13
    public static Pose scoreSubmersible3ControlPoint = new Pose(62.649350649350644, 121.79220779220779, Math.toRadians(-15));


    public PathChain changeHeadingSubmersible1;
    public PathChain changeHeadingSubmersible2;
    public PathChain changeHeadingSubmersible3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.intake.desiredSampleColor = Intake.DesiredSampleColor.BOTH;
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
        robot.limelightCamera.initializeHardware(hardwareMap);
        robot.limelightCamera.initializeBLUE();

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
                .setLinearHeadingInterpolation(score3Pose.getHeading(), submersible1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

//        PathChain scoreSubmersible1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
//                .setLinearHeadingInterpolation(submersible1Pose.getHeading(), scoreSubmersible1Pose.getHeading())
//                .build();
        PathChain scoreSubmersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible1Pose), new Point(scoreSubmersible1ControlPoint), new Point(scoreSubmersible1Pose)))
                .setLinearHeadingInterpolation(submersible1Pose.getHeading(), scoreSubmersible1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        PathChain submersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSubmersible1Pose), new Point(submersible2ControlPoint), new Point(submersible2Pose)))
                .setLinearHeadingInterpolation(scoreSubmersible1Pose.getHeading(), submersible2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        PathChain scoreSubmersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible2Pose), new Point(scoreSubmersible2ControlPoint), new Point(scoreSubmbersible2Pose)))
                .setLinearHeadingInterpolation(submersible2Pose.getHeading(), scoreSubmbersible2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        PathChain submersible3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSubmbersible2Pose), new Point(submersible3ControlPoint), new Point(submersible3Pose)))
                .setLinearHeadingInterpolation(scoreSubmbersible2Pose.getHeading(), submersible3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();


        PathChain scoreSubmersible3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(submersible3Pose), new Point(scoreSubmersible3ControlPoint), new Point(scoreSubmbersible3Pose)))
                .setLinearHeadingInterpolation(submersible3Pose.getHeading(), scoreSubmbersible3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();


        changeHeadingSubmersible1 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        changeHeadingSubmersible2 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        changeHeadingSubmersible3 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible3Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();




//        while(opModeInInit()){
//            follower.update();
//        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SetClawStateCommand(Arm.ClawState.OPEN), //don't ask
                        new FollowPath(follower, scorePreload, true, 1),
                        new FollowPath(follower, grab1, true, 1),
                        new FollowPath(follower, score1, true, 1),
                        new FollowPath(follower, grab2, true, 1),
                        new FollowPath(follower, score2, true, 1),
                        new FollowPath(follower, grab3, true, 1),
                        new FollowPath(follower, score3, true, 1),


                        ///////////////////////////////////SUBMERSIBLE 1/////////////////////////////////////////
                        new FollowPath(follower, submersible1, true, 1),
                        new WaitCommand(300),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(this::runLimelightHeadingPath_1),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_POSE),
                        new WaitUntilCommand(robot.extendo::limelightPoseFinished),

                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitCommand(100),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_RETRACT_POSE).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.extendo::limelightRetractPoseFinished).interruptOn(robot.intake::isSampleDigital),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.extendo::limelightTakePoseFinished).interruptOn(robot.intake::isSampleDigital),

                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(1000),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),



                        new FollowPath(follower, scoreSubmersible1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoCommand(),
                                                new WaitCommand(500),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),
                        new ScoreSampleAutoCommand(),


                        /////////////////////////////////SUBMERSIBLE 2/////////////////////////////////////////
                        new FollowPath(follower, submersible2, true,  1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand()
                                ),
                        new WaitCommand(300),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(this::runLimelightHeadingPath_2),

                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_POSE),
                        new WaitUntilCommand(robot.extendo::limelightPoseFinished),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitCommand(100),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_RETRACT_POSE).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.extendo::limelightRetractPoseFinished).interruptOn(robot.intake::isSampleDigital),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.extendo::limelightTakePoseFinished).interruptOn(robot.intake::isSampleDigital),

                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(1000),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new FollowPath(follower, scoreSubmersible2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoCommand(),
                                                new WaitCommand(500),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),
                        new ScoreSampleAutoCommand(),


                        ///////////////////////////////////SUBMERSIBLE 3/////////////////////////////////////////
                        new FollowPath(follower, submersible3, true,  1)
                                .alongWith(
                                        new OuttakeGoBackToIdleFromHighBasketCommand()
                                ),
                        new WaitCommand(300),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(()->robot.limelightCamera.upadateLimelightPythonSnap()),
                        new InstantCommand(this::runLimelightHeadingPath_3),

                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_POSE),
                        new WaitUntilCommand(robot.extendo::limelightPoseFinished),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new WaitCommand(100),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_RETRACT_POSE).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.extendo::limelightRetractPoseFinished).interruptOn(robot.intake::isSampleDigital),
                        new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE).interruptOn(robot.intake::isSampleDigital),
                        new WaitUntilCommand(robot.extendo::limelightTakePoseFinished).interruptOn(robot.intake::isSampleDigital),

                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(1000),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),

                        new FollowPath(follower, scoreSubmersible3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractBASKETAutoCommand(),
                                                new WaitCommand(500),
                                                new OuttakeGoHighBasketAutoCommand()
                                        )
                                ),
                        new ScoreSampleAutoCommand(),
                        new OuttakeGoBackToIdleFromHighBasketCommand()
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


            telemetry.addData("y_distance",robot.limelightCamera.y_distance);
            telemetry.addData("x_distance", robot.limelightCamera.x_distance);
            telemetry.addData("extendoDistance", robot.limelightCamera.extendoDistance);
            telemetry.addData("targetAngle", robot.limelightCamera.targetAngle);
            telemetry.addData("EXTENDO TARGET POS:", robot.extendo.getTargetPosition());
            telemetry.addData("EXTENDO CUR POS:", robot.extendo.currentPosition);
            telemetry.addData("EXTEMDO STATE:", robot.extendo.extendoState);
            telemetry.addData("GLOBALS EXTENDO DISTANCE:", Globals.extendoDistance);
            telemetry.update();

            double loop = System.nanoTime();
            telemetry.addData("Hz", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

        }

    }



    public void runLimelightHeadingPath_1(){
        changeHeadingSubmersible1 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible1Pose)))
                .setConstantHeadingInterpolation(robot.limelightCamera.targetAngle + Math.toRadians(-90) - Math.toRadians(0.9))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, changeHeadingSubmersible1, true, 0.8)
        );
    }


    public void runLimelightHeadingPath_2(){
        changeHeadingSubmersible2 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible2Pose)))
                .setConstantHeadingInterpolation(robot.limelightCamera.targetAngle + Math.toRadians(-90) - Math.toRadians(0.9))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, changeHeadingSubmersible2, true, 0.8)
        );
    }


    public void runLimelightHeadingPath_3(){
        changeHeadingSubmersible3 = follower.pathBuilder()
                .addPath(new BezierPoint(new Point(submersible3Pose)))
                .setConstantHeadingInterpolation(robot.limelightCamera.targetAngle + Math.toRadians(-90) - Math.toRadians(0.9))
                .build();

        CommandScheduler.getInstance().schedule(
                new FollowPath(follower, changeHeadingSubmersible3, true, 0.8)
        );
    }
}
