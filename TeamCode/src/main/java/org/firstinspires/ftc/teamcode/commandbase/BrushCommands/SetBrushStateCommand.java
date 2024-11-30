package org.firstinspires.ftc.teamcode.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Brush;

public class SetBrushStateCommand extends InstantCommand {
    public SetBrushStateCommand(Brush.BrushState state){
        super(
                () -> Brush.getInstance().setState(state)
        );
    }
}
