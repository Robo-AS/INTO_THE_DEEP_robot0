package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.SpecimenAuto;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
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
                new SetIntakeStateCommand(Intake.IntakeState.OUTTAKING_SPECIMEN),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingSPECIMEN_1_AUTO),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSpecimen_AUTO),
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetIntakeStateCommand(Intake.IntakeState.OUTTAKING_SPECIMEN),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingSPECIMEN_2_AUTO).withTimeout(1000),
//                new WaitUntilCommand(Intake.getInstance()::canCloseClaw_AUTO),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
//                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingSPECIMEN_2_AUTO),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE)

        );
    }
}
