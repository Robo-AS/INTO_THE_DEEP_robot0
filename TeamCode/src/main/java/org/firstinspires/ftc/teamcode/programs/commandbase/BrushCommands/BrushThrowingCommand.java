package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushThrowingCommand extends ParallelCommandGroup {
    public BrushThrowingCommand(){
        super(
                new BrushCommand(0, 1),
                new SetBrushStateCommand(Brush.BrushState.THROWING)
        );
    }
}
