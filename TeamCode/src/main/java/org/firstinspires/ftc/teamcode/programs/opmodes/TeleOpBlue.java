package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeIntakingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.tests.LiftTests.SetLiftStateCommandTEST;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue", group = "OpModes")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;


    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);


        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();


        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.YELLOW));


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BLUE));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BOTH));



        //spitting button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(
                                () -> CommandScheduler.getInstance().schedule(
                                        new SequentialCommandGroup(
                                                new ConditionalCommand(
                                                        new InstantCommand(()-> robot.brush.updatePreviousBrushState(robot.brush.brushState)),
                                                        new DoesNothingCommand(),
                                                        () -> robot.brush.brushState != Brush.BrushState.SPITTING_HUMAN_PLAYER
                                                ),
                                                new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER)
                                        )
                                )
                        );
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(new SetBrushStateCommand(robot.brush.previousBrushState));




        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new IntakeIntakingCommand(),
                                                new DoesNothingCommand(),
                                                () -> robot.extendo.extendoState == Extendo.ExtendoState.EXTENDING_MINIMUM

                                        ),
                                        new IntakeIdleCommand(),
                                        () -> robot.brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );


        Trigger extendoTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        extendoTrigger
                .whenActive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM),
                                        new ConditionalCommand(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new IntakeRetractCommand(),
                                                () -> robot.brush.brushAngle == Brush.BrushAngle.UP
                                        ),
                                        () -> robot.extendo.extendoState == Extendo.ExtendoState.RETRACTING
                                )
                        )
                );


        //Lift commands
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetLiftStateCommandTEST(Lift.LiftState.HIGH_RUNG),
                                        new SetLiftStateCommandTEST(Lift.LiftState.IDLE),
                                        () -> robot.lift.liftState == Lift.LiftState.IDLE
                                )
                        )

                );

        Trigger liftTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        liftTrigger
                .whenActive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetLiftStateCommandTEST(Lift.LiftState.HIGH_BASKET),
                                        new SetLiftStateCommandTEST(Lift.LiftState.IDLE),
                                        () -> robot.lift.liftState == Lift.LiftState.IDLE
                                )
                        )
                );


        //balaceala de button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new OuttakeCommand(),
                                                new DoesNothingCommand(),
                                                () -> robot.brush.sampleState == Brush.SampleState.IS && robot.brush.intakedSampleColor == Brush.IntakedSampleColor.BLUE
                                        ),
                                        new DoesNothingCommand(),
                                        () -> robot.extendo.extendoState == Extendo.ExtendoState.RETRACTING
                                )
                        )
                );

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        robot.brush.loop();
        robot.extendo.loop(gamepadEx.getLeftY());
        robot.lift.loop();


        double turnSpeed = -gamepad1.left_stick_x;
        Pose drive = new Pose(-gamepad1.right_stick_x, -gamepad1.right_stick_y, turnSpeed);
        robot.mecanumDriveTrain.set(drive, 0);

//        telemetry.addData("BrushState:", robot.brush.brushState);
//        telemetry.addData("PreviousBrushState:", robot.brush.previousBrushState);
//        telemetry.addData("DesiredColor", robot.brush.desiredSampleColor);
//        telemetry.addData("IntakedColor:", robot.brush.intakedSampleColor);
//        telemetry.addData("SampleState:", robot.brush.sampleState);
//        telemetry.addData("AngleServoPosition", robot.brush.brushAngleServo.getPosition());
//        telemetry.addData("Brush Angle", robot.brush.brushAngle);

        telemetry.addData("Current Position", robot.extendo.extendoMotor.getCurrentPosition());
        telemetry.addData("Target Position", robot.extendo.getTargetPosition());
        telemetry.addData("Extendo State", robot.extendo.extendoState);

//        telemetry.addData("Joystick Y", gamepadEx.gamepad.left_stick_y);
//        telemetry.addData("Joystick Y MODIFIED", robot.extendo.getExponentialJoystickCoef());

//        telemetry.addData("Current Position", robot.lift.liftMotor.getCurrentPosition());
//        telemetry.addData("Target Position", robot.lift.getTargetPosition());
        telemetry.update();

    }
}
