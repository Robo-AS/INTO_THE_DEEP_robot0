package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;


@TeleOp(name="TELEOP TEST")
public class TeleOpTest extends CommandOpMode {
    private GamepadEx gamepadEx;
    private Brush brush;
    @Override
    public void initialize(){

        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetBrushStateCommandTEST(Brush.BrushState.INTAKING, brush));

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        brush.updateIntakedSampleColor();

        telemetry.addData("BrushState:", brush.brushState);
        telemetry.update();



    }
}
