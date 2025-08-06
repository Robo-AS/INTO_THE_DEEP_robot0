package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeBlockedSamplesCommand extends SequentialCommandGroup {
    public IntakeBlockedSamplesCommand(){
        super(
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new WaitCommand(50),
                new SetIntakeStateCommand(Intake.IntakeState.INTAKING)
        );
    }
}
