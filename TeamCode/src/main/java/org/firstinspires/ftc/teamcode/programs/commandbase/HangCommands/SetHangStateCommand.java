package org.firstinspires.ftc.teamcode.programs.commandbase.HangCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;

public class SetHangStateCommand extends InstantCommand {
    public SetHangStateCommand(Hang.HangState state){
        super(
                () -> Hang.getInstance().updateHang(state)
        );
    }
}
