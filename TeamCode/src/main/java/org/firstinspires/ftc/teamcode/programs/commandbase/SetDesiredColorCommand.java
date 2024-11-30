package org.firstinspires.ftc.teamcode.programs.commandbase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class SetDesiredColorCommand extends InstantCommand {
    public SetDesiredColorCommand(Brush.DesiredSampleColor color){
        super(
                () -> Brush.getInstance().updateDesiredSampleColor(color)
        );
    }
}
