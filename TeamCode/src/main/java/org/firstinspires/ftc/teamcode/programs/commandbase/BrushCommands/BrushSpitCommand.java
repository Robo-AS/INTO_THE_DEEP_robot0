package org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;

public class BrushSpitCommand extends InstantCommand {
    public BrushSpitCommand(){
        super(
                () -> Brush.getInstance().brushMotor.setPower(-1)
        );
    }
}
