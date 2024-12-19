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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ATeleOpBlue")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private GamepadEx gamepadEx;


    @Override
    public void initialize(){
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

        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE),
                                        () -> robot.brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();


        if(robot.brush.brushState == Brush.BrushState.IDLE){
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
        }

        if(robot.brush.brushState == Brush.BrushState.INTAKING && robot.brush.sampleState == Brush.SampleState.ISNOT){
            robot.brush.updateSampleState();
            robot.brush.updateIntakedSampleColor();
            CommandScheduler.getInstance().schedule(new BrushIntakeCommand());
        }

        else if(robot.brush.brushState == Brush.BrushState.INTAKING && robot.brush.sampleState == Brush.SampleState.IS){
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            while(robot.brush.intakedSampleColor == Brush.IntakedSampleColor.NOTHING){
                robot.brush.updateIntakedSampleColor();
            }

            if(robot.brush.isRightSampleColorTeleOpBlue()){
                CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            }
            else{
                CommandScheduler.getInstance().schedule(new BrushThrowingCommand());
            }
        }


        if(robot.brush.brushState == Brush.BrushState.THROWING && robot.brush.sampleState == Brush.SampleState.IS){
            robot.brush.updateSampleState();
        }


        if(robot.brush.brushState == Brush.BrushState.THROWING && robot.brush.sampleState == Brush.SampleState.ISNOT){
            CommandScheduler.getInstance().schedule(new SetBrushStateCommand(Brush.BrushState.INTAKING));
        }

        telemetry.addData("BrushState:", robot.brush.brushState);
        telemetry.addData("DesiredColor", robot.brush.desiredSampleColor);
        telemetry.addData("IntakedColor:", robot.brush.intakedSampleColor);
        telemetry.addData("SampleState:", robot.brush.sampleState);

        telemetry.update();

    }
}
