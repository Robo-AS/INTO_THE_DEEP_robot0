package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.HangCommands.SetSafetyStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.NEWSetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetSweeperStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands.GoHangLevel2Position;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands.TriggerHangCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeIntakingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeRetractCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWOuttakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighBasketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighBascketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoLowBasketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakeGoBackToIdleFromLowBasketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakePutSampleHighBasketGoBackToIdle;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakePutSampleLowBasketGoBackToIdle;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.PutSpecimenCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;

import java.util.function.BooleanSupplier;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "🟦NEWTeleOpBlue_WITHAUTO🟦", group = "OpModes")
public class NEWTeleOpBlue_WITHAUTO extends CommandOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    public GamepadEx gamepadEx;
    private Follower follower;

    private double loopTime = 0;
    public boolean AUTO_IN_TELEOP = false;


    private final Pose startPose = new Pose(40.2077922077922, 65.92207792207792, Math.toRadians(180));
    private final Pose goForward = new Pose(27.2077922077922, 65.92207792207792, Math.toRadians(180));

    private PathChain go;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);


        gamepadEx = new GamepadEx(gamepad1);

        AUTO_IN_TELEOP = false;




        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();



        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new InstantCommand(this::initializeAUTO_IN_TELEOP)
                                                ),

                                                new SequentialCommandGroup(
                                                        new InstantCommand(() -> follower.breakFollowing()),
//                                                        new InstantCommand(() -> CommandScheduler.getInstance().reset()),
                                                        new InstantCommand(() -> AUTO_IN_TELEOP = false)

                                                ),
                                                ()-> !AUTO_IN_TELEOP
                                        )

                                )
                        )
                );





    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        if(AUTO_IN_TELEOP){
            follower.update();
        }
        else{
            telemetry.addData("NO FOLLOWER", "NO");
        }



        telemetry.addData("AUTO_IN_TELEOP", AUTO_IN_TELEOP);
        telemetry.addData("SampleColor", robot.intake.intakedSampleColor);
        telemetry.addData("e?", robot.intake.sampleState);

        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }




    public void initializeAUTO_IN_TELEOP(){

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        go = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(goForward)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goForward.getHeading())
                .build();



        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> AUTO_IN_TELEOP = true),
                        new WaitCommand(1000),
                        new FollowPath(follower, go, true, 0.5),
                        new InstantCommand(() -> AUTO_IN_TELEOP = false),
                        new InstantCommand(() -> follower.breakFollowing())

                )

        );


    }

    public void reInitializeMecanum(){
        robot.mecanumDriveTrain.initializeHardware(hardwareMap);
        robot.mecanumDriveTrain.initialize();
    }

    public void runAuto(){


    }
}
