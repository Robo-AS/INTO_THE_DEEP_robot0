package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.IntakeTests.SetBrushStateCommandTEST;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIntakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushThrowingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlueTESTTT")
public class TeleOpBlueTest extends CommandOpMode {
    private Brush brush = Brush.getInstance();
    private GamepadEx gamepadEx;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        brush.initializeHardware(hardwareMap);
        brush.initialize();




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
                                        () -> brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        brush.updateIntakedSampleColor();


        if(brush.brushState == Brush.BrushState.IDLE){
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
        }

        if(brush.brushState == Brush.BrushState.INTAKING && brush.sampleState == Brush.SampleState.ISNOT){
            brush.updateSampleState();
            CommandScheduler.getInstance().schedule(new BrushIntakeCommand());
        }

        else if(brush.brushState == Brush.BrushState.INTAKING && brush.sampleState == Brush.SampleState.IS){
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            while(brush.intakedSampleColor == Brush.IntakedSampleColor.NOTHING){
                brush.updateIntakedSampleColor();
            }

            if(brush.isRightSampleColorTeleOpBlue()){
                CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            }
            else{
                CommandScheduler.getInstance().schedule(new BrushThrowingCommand());
            }
        }


        if(brush.brushState == Brush.BrushState.THROWING && brush.sampleState == Brush.SampleState.IS){
            brush.updateSampleState();
        }


        if(brush.brushState == Brush.BrushState.THROWING && brush.sampleState == Brush.SampleState.ISNOT){
            CommandScheduler.getInstance().schedule(new SetBrushStateCommand(Brush.BrushState.INTAKING));
        }

        telemetry.addData("BrushState:", brush.brushState);
        telemetry.addData("DesiredColor", brush.desiredSampleColor);
        telemetry.addData("IntakedColor:", brush.intakedSampleColor);
        telemetry.addData("SampleState:", brush.sampleState);

        telemetry.update();
    }
}
