package org.firstinspires.ftc.teamcode.programs.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.IntakeRetractAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.opencv.core.Mat;


@Config
@Autonomous(name = "SpecimenAuto")
public class SpecimenAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private Follower follower;
    private double loopTime = 0;



    public static Pose startPose = new Pose(8, 55, Math.toRadians(180));
    public static Pose preloadPose = new Pose(37, 72, Math.toRadians(180));

    public static Pose grab1Pose = new Pose(30, 40, Math.toRadians(-45));


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


        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setConstantHeadingInterpolation(preloadPose.getHeading())
                .build();



        PathChain grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(grab1Pose)))
                .setTangentHeadingInterpolation()
                .build();



        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, scorePreload, true, 1),
                        new WaitCommand(500),
                        new FollowPath(follower, grab1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(700),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO),
                                                new WaitCommand(500),
                                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                                        )

                                ),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SAMPLE_SPECIMEN),
                        new WaitUntilCommand(robot.brush::isSample),
                        new SetBrushStateCommand(Brush.BrushState.IDLE),
                        new IntakeRetractAutoCommand()

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


        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();


    }
}
