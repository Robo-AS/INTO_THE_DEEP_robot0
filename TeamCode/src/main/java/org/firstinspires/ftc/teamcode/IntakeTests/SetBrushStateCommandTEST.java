package org.firstinspires.ftc.teamcode.IntakeTests;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class SetBrushStateCommandTEST extends InstantCommand {
    public SetBrushStateCommandTEST(Brush.BrushState state, Brush brush){
        super(
                () -> brush.updateBrushState(state)
        );
    }
}
