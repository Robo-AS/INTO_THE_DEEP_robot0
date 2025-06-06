package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

public class IntakeRetractFailSafeCommand extends SequentialCommandGroup {
    public IntakeRetractFailSafeCommand() {
        super(
                new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACT_AUTO_FAIL_SAFE)

        );
    }
}
