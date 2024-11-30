package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class SetBrushStateCommand extends InstantCommand {
    public SetBrushStateCommand(Brush.BrushState state){
        super(
                () -> Brush.getInstance().updateBrushState(state)
        );
    }
}
