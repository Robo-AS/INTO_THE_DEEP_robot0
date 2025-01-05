package org.firstinspires.ftc.teamcode.tests.dontneedanymore;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushIdleCommand extends ParallelCommandGroup {
    public BrushIdleCommand(){
        super(
                new BrushCommand(0, 0.5),
                new SetBrushStateCommand(Brush.BrushState.IDLE)
        );
    }
}
