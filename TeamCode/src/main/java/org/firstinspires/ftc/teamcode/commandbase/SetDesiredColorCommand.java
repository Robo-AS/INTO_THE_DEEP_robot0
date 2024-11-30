package org.firstinspires.ftc.teamcode.commandbase;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Brush;

public class SetDesiredColorCommand extends InstantCommand {
    public SetDesiredColorCommand(Brush.DesiredSampleColor color){
        super(
                () -> Brush.getInstance().setDesiredSampleColor(color)
        );
    }
}
