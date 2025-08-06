package org.firstinspires.ftc.teamcode.programs.commandbase.HangCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;

public class SetSafetyStateCommand extends InstantCommand {
    public SetSafetyStateCommand(Hang.SafetyState state){
        super(
                () -> Hang.getInstance().updateSafety(state)
        );
    }
}
