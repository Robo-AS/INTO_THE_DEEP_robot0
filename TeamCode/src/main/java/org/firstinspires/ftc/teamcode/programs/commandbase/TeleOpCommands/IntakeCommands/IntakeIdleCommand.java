package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class IntakeIdleCommand extends ParallelCommandGroup {
    public IntakeIdleCommand(){
        super(
                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                new SetJoystickConstantCommand(Globals.EXTENDO_JOYSTICK_CONSTANT_UP),
                new SetBrushStateCommand(Brush.BrushState.IDLE)
        );

    }
}
