package org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class SetLiftStateCommand extends InstantCommand {
    public SetLiftStateCommand(Lift.LiftState state){
        super(
                () -> Lift.getInstance().update(state)
        );
    }
}
