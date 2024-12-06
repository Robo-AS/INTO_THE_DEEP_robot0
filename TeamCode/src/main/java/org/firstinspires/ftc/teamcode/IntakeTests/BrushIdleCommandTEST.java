package org.firstinspires.ftc.teamcode.IntakeTests;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushIdleCommandTEST extends ParallelCommandGroup {
    public BrushIdleCommandTEST(Brush brush){
        super(
                new BrushCommandTEST(0, 0.5, brush),
                new SetBrushStateCommandTEST(Brush.BrushState.IDLE, brush)
        );
    }
}
