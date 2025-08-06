package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class GoHangLevel2Position extends SequentialCommandGroup {
    public GoHangLevel2Position(){
        super(
                new SetLiftStateCommand(Lift.LiftState.HANG),
                new SetExtendoStateCommand(Extendo.ExtendoState.HANG),
                new InstantCommand(Globals::isHangingLevel2)
        );
    }
}
