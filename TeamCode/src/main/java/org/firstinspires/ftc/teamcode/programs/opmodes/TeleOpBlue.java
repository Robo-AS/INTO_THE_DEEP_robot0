package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIntakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushSpitCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushThrowingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeIntakingCommand;
import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue", group = "OpModes")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();

//        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                        .whenPressed(
//                                () -> CommandScheduler.getInstance().schedule(
//                                        new InstantCommand(()-> robot.brush.updatePreviousBrushState(robot.brush.brushState)),
//                                        new SetBrushStateCommand(Brush.BrushState.SPITTING)
//                                )
//                        );
//        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenReleased(new SetBrushStateCommand(robot.brush.previousBrushState));


        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.YELLOW));


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BLUE));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BOTH));

        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new IntakeIntakingCommand(),
                                        new IntakeIdleCommand(),
                                        () -> robot.brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        robot.brush.loop();

        telemetry.addData("BrushState:", robot.brush.brushState);
        telemetry.addData("DesiredColor", robot.brush.desiredSampleColor);
        telemetry.addData("IntakedColor:", robot.brush.intakedSampleColor);
        telemetry.addData("SampleState:", robot.brush.sampleState);
        telemetry.addData("AngleServoPosition", robot.brush.brushAngleServo.getPosition());
        telemetry.update();

    }
}
