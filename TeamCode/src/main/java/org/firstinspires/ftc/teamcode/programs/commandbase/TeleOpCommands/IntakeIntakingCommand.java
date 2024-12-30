package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIntakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class IntakeIntakingCommand extends ParallelCommandGroup {
    public IntakeIntakingCommand(){
        super(
                new SetBrushAngleCommand(Globals.BRUSH_POSITION_DOWN),
                new SetBrushStateCommand(Brush.BrushState.INTAKING)
        );

    }
}
