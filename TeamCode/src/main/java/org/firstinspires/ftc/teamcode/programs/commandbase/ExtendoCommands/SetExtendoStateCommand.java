package org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

public class SetExtendoStateCommand extends InstantCommand {
    public SetExtendoStateCommand(Extendo.ExtendoState state){
        super(
                () -> Extendo.getInstance().update(state)
        );
    }
}
