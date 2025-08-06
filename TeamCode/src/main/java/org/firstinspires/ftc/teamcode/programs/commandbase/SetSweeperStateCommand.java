package org.firstinspires.ftc.teamcode.programs.commandbase;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;

public class SetSweeperStateCommand extends InstantCommand {
    public SetSweeperStateCommand(Sweeper.SweeperState state){
        super(
                () -> Sweeper.getInstance().update(state)
        );
    }
}
