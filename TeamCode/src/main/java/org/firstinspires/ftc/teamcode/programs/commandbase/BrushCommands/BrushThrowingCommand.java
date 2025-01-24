package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class BrushThrowingCommand extends ParallelCommandGroup {
    public BrushThrowingCommand(){
        super(
                new BrushCommand(Globals.BRUSH_MOTOR_SPEED, 1),
                new SetBrushStateCommand(Brush.BrushState.THROWING)
        );
    }
}
