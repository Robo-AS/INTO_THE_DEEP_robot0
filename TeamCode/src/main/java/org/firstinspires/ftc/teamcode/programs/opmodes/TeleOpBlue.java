package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIntakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushThrowingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = new Robot();
    private GamepadEx gamepadEx;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeRobotHadrware();

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
                                        new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE),
                                        () -> robot.intake.brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();




        if(robot.intake.brush.brushState == Brush.BrushState.INTAKING && robot.intake.brush.sampleState == Brush.SampleState.ISNOT){
            robot.intake.brush.updateSampleState();
            CommandScheduler.getInstance().schedule(new BrushIntakeCommand());
        }

        else if(robot.intake.brush.brushState == Brush.BrushState.INTAKING && robot.intake.brush.sampleState == Brush.SampleState.IS){
            robot.intake.brush.updateIntakedSampleColor();
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            if(robot.intake.brush.isRightSampleColorBlue()){
                CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            }
            else CommandScheduler.getInstance().schedule(new BrushThrowingCommand());
        }

        if(robot.intake.brush.brushState == Brush.BrushState.THROWING)
            robot.intake.brush.updateSampleState();

        if(robot.intake.brush.brushState == Brush.BrushState.THROWING && robot.intake.brush.sampleState == Brush.SampleState.ISNOT){
            CommandScheduler.getInstance().schedule(new SetBrushStateCommand(Brush.BrushState.INTAKING));
        }



    }
}
