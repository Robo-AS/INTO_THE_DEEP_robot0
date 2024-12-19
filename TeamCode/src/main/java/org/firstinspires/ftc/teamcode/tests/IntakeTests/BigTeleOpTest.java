package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BigTeleOpTest")
public class BigTeleOpTest extends CommandOpMode {
    private Brush brush;
    private GamepadEx gamepadEx;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);
        brush = new Brush();

        brush.initializeHardware(hardwareMap);

        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetDesiredColorCommandTEST(Brush.DesiredSampleColor.YELLOW, brush));


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetDesiredColorCommandTEST(Brush.DesiredSampleColor.BLUE, brush));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetDesiredColorCommandTEST(Brush.DesiredSampleColor.BOTH, brush));

        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetBrushStateCommandTEST(Brush.BrushState.INTAKING, brush),
                                        new SetBrushStateCommandTEST(Brush.BrushState.IDLE, brush),
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
            CommandScheduler.getInstance().schedule(new BrushIdleCommandTEST(brush));
        }

        if(brush.brushState == Brush.BrushState.INTAKING && brush.sampleState == Brush.SampleState.ISNOT){
            brush.updateSampleState();
            CommandScheduler.getInstance().schedule(new BrushIntakeCommandTEST(brush));
        }

        else if(brush.brushState == Brush.BrushState.INTAKING && brush.sampleState == Brush.SampleState.IS){
            CommandScheduler.getInstance().schedule(new BrushIdleCommandTEST(brush));
            while(brush.intakedSampleColor == Brush.IntakedSampleColor.NOTHING){
                brush.updateIntakedSampleColor();
            }

            if(brush.isRightSampleColorTeleOpBlue()){
                CommandScheduler.getInstance().schedule(new BrushIdleCommandTEST(brush));
            }
            else{
                CommandScheduler.getInstance().schedule(new BrushThrowingCommandTEST(brush));
            }
        }


        if(brush.brushState == Brush.BrushState.THROWING && brush.sampleState == Brush.SampleState.IS){
            brush.updateSampleState();
        }


        if(brush.brushState == Brush.BrushState.THROWING && brush.sampleState == Brush.SampleState.ISNOT){
            CommandScheduler.getInstance().schedule(new SetBrushStateCommandTEST(Brush.BrushState.INTAKING, brush));
        }

        telemetry.addData("BrushState:", brush.brushState);
        telemetry.addData("DesiredColor", brush.desiredSampleColor);
        telemetry.addData("IntakedColor:", brush.intakedSampleColor);
        telemetry.addData("SampleState:", brush.sampleState);

        telemetry.update();
    }
}
