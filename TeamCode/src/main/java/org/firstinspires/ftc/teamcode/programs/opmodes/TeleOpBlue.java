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
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeIntakingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighBascketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakePutSampleGoBackToIdle;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.PutSpecimenCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue", group = "OpModes")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    public GamepadEx gamepadEx;

    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
    public static double contantTerm = 0.6, liniarCoefTerm = 0.7;
    private double loopTime = 0;

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
                                        new OuttakeGoHighRungCommand(),
                                        new ConditionalCommand(
                                                new PutSpecimenCommand(),
                                                new ConditionalCommand(
                                                        new OuttakeGoBackToIdleCommand(),
                                                        new DoesNothingCommand(),
                                                        () -> robot.arm.clawState == Arm.ClawState.OPEN
                                                )                                                                                                                                                                                                              ,
                                                () -> robot.arm.clawState == Arm.ClawState.CLOSED
                                        ),
                                        () -> robot.lift.liftState == Lift.LiftState.IDLE
                                )
                        )

                );



        Trigger liftTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        liftTrigger
                .whenActive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new OuttakeGoHighBascketCommand(),
                                        new ConditionalCommand(
                                                new OutttakePutSampleGoBackToIdle(),
                                                new OuttakeGoBackToIdleCommand(),
                                                () -> robot.arm.clawState == Arm.ClawState.CLOSED
                                        ),
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

        robot.loop();
        robot.brush.loop();
        robot.extendo.loop(gamepadEx.getLeftY());
        robot.lift.loop();
        robot.arm.loop();

        //applying expo function for mecanum
        exponentialJoystickCoord_X_TURN = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * contantTerm;
        exponentialJoystickCoord_X_FORWARD = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * contantTerm;
        exponentialJoystickCoord_Y = (Math.pow(gamepad1.right_stick_y, 3) + liniarCoefTerm * gamepad1.right_stick_y) * contantTerm;

        double turnSpeed = robot.extendo.extendoState == Extendo.ExtendoState.EXTENDING_MINIMUM ? -exponentialJoystickCoord_X_TURN/2 :-exponentialJoystickCoord_X_TURN;
        Pose drive = new Pose(-exponentialJoystickCoord_X_FORWARD, -exponentialJoystickCoord_Y, turnSpeed);
        robot.mecanumDriveTrain.set(drive, 0);






        telemetry.addData("BrushState:", robot.brush.brushState);
        telemetry.addData("PreviousBrushState:", robot.brush.previousBrushState);
//        telemetry.addData("DesiredColor", robot.brush.desiredSampleColor);
//        telemetry.addData("IntakedColor:", robot.brush.intakedSampleColor);
//        telemetry.addData("SampleState:", robot.brush.sampleState);
//        telemetry.addData("AngleServoPosition", robot.brush.brushAngleServo.getPosition());
//        telemetry.addData("Brush Angle", robot.brush.brushAngle);

//        telemetry.addData("Current Position", robot.extendo.extendoMotor.getCurrentPosition());
//        telemetry.addData("Target Position", robot.extendo.getTargetPosition());
//        telemetry.addData("Extendo State", robot.extendo.extendoState);
//        telemetry.addData("Joystick Y", gamepadEx.gamepad.left_stick_y);
//        telemetry.addData("Joystick Y MODIFIED", robot.extendo.getExponentialJoystickCoef());

//        telemetry.addData("Current Position", robot.lift.liftMotor.getCurrentPosition());
//        telemetry.addData("Target Position", robot.lift.getTargetPosition());


        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}
