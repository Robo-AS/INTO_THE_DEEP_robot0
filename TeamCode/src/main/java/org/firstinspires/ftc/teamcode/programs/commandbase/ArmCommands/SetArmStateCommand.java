package org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

public class SetArmStateCommand extends InstantCommand {
    public SetArmStateCommand(Arm.ArmState state){
        super(
                () -> Arm.getInstance().update(state)
        );
    }
}
