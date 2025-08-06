package org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

public class SetClawStateCommand extends InstantCommand {
    public SetClawStateCommand(Arm.ClawState state){
        super(
                () -> Arm.getInstance().update(state)
        );
    }
}
