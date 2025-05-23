package org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

public class SetJoystickConstantCommand extends InstantCommand {
    public SetJoystickConstantCommand(double constant){
        super(
                () -> Extendo.getInstance().updateJoystickConstant(constant)
        );
    }
}
