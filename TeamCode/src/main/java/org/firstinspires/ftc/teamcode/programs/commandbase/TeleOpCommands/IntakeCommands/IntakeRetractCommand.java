package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(){
        super(
            new SetBrushStateCommand(Brush.BrushState.IDLE),
            new SetBrushAngleCommand(Brush.BrushAngle.UP),
            new SetJoystickConstantCommand(Globals.EXTENDO_JOYSTICK_CONSTANT_UP),
            new SetBrushStateCommand(Brush.BrushState.SPITTING),
//            new WaitCommand(150),//300
            new SetBrushStateCommand(Brush.BrushState.IDLE),
            new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
        );
    }
}
