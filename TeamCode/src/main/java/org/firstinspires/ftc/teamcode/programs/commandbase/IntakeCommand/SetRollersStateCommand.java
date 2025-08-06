package org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;


public class SetRollersStateCommand extends InstantCommand {
    public SetRollersStateCommand(Intake.RollersState state){
        super(
                () -> Intake.getInstance().updateRollersState(state)
        );
    }
}
