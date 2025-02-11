package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.HangCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class GoHangLevel2Position extends SequentialCommandGroup {
    public GoHangLevel2Position(){
        super(
                new SetLiftStateCommand(Lift.LiftState.HIGH_BASKET),
                new SetExtendoStateCommand(Extendo.ExtendoState.HANG)
        );
    }
}
