package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.BasketAuto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetRollersStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeRetractBASKETAutoSUBMERSIBLECommand extends SequentialCommandGroup {
    public IntakeRetractBASKETAutoSUBMERSIBLECommand() {
        super(
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new WaitCommand(100),
                new SetBrushStateCommand(Intake.BrushState.IDLE),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingYELLOW_1_AUTO_SUBMERSIBLE),
                new SetRollersStateCommand(Intake.RollersState.IDLE),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canCloseClaw_AUTO_SUBMERSIBLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingYELLOW_2_AUTO_SUBMERSIBLE),
                new SetRollersStateCommand(Intake.RollersState.IDLE)
        );
    }
}
