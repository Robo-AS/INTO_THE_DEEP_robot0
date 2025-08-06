package org.firstinspires.ftc.teamcode.tests.IntakeTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


@Disabled
@TeleOp(name = "IntakeTeleOpTest", group = "Tests")
public class IntakeTeleOpTest extends CommandOpMode {
    private final Intake intake = Intake.getInstance();
    public GamepadEx gamepadEx;




    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        intake.initializeHardware(hardwareMap);
        intake.initialize();


        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetIntakeStateCommand(Intake.IntakeState.IDLE));


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SetIntakeStateCommand(Intake.IntakeState.INTAKING));




        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER));








    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        intake.loopBlue();
        telemetry.addData("IntakedColor:", intake.intakeState);
        telemetry.addData("Colour:", intake.intakedSampleColor);
        telemetry.addData("State:", intake.sampleState);
        telemetry.addData("DesiredColour", intake.desiredSampleColor);
        telemetry.update();
    }
}
