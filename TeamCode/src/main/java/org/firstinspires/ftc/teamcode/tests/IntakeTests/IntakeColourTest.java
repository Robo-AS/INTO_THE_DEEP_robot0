package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


@Disabled
@TeleOp(name="IntakeColourTest", group = "Tests")
public class IntakeColourTest extends CommandOpMode {
    private GamepadEx gamepadEx;
    private final Intake intake = Intake.getInstance();

    @Override
    public void initialize(){
        intake.initializeHardware(hardwareMap);
//        CommandScheduler.getInstance().reset();
//
//        gamepadEx = new GamepadEx(gamepad1);
//
//        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new SetBrushStateCommandTEST(Brush.BrushState.INTAKING, brush));

    }

    @Override
    public void run(){
        //CommandScheduler.getInstance().run();
        intake.updateSampleColor();
        intake.updateSampleStateDigital();

        telemetry.addData("Color:", intake.intakedSampleColor);
        telemetry.addData("State:", intake.sampleState);
        telemetry.update();



    }
}
