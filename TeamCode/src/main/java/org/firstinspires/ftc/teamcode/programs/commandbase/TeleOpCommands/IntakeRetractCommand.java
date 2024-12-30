package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.tests.ExtendoTests.SetExtendoStateCommandTEST;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(){
        super(
            new BrushIdleCommand(),
            new SetBrushAngleCommand(Brush.BrushAngle.UP),
            new SetBrushStateCommand(Brush.BrushState.SPITTING),
            new WaitCommand(300),
            new SetBrushStateCommand(Brush.BrushState.IDLE),
            new SetExtendoStateCommandTEST(Extendo.ExtendoState.RETRACTING)

        );
    }
}
