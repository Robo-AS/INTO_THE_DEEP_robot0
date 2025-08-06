package org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

public class SetWristStateCommand extends InstantCommand {
    public SetWristStateCommand(Arm.WristState state){
        super(
                () -> Arm.getInstance().update(state)
        );
    }
}
