package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.SpecimenAuto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetRollersStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeRetractSPECIMENAutoCommand extends SequentialCommandGroup {
    public IntakeRetractSPECIMENAutoCommand() {
        super(
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingSPECIMEN_1_AUTO),
                new SetRollersStateCommand(Intake.RollersState.IDLE),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canCloseClaw_AUTO),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingSPECIMEN_2_AUTO),
                new SetRollersStateCommand(Intake.RollersState.IDLE)

        );
    }
}
