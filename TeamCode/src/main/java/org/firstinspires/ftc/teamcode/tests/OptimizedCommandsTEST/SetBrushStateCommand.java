package org.firstinspires.ftc.teamcode.tests.OptimizedCommandsTEST;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class SetBrushStateCommand extends InstantCommand {
    public SetBrushStateCommand(Brush.BrushState state){
        super(
                () -> Brush.getInstance().updateState(state)
        );
    }
}
