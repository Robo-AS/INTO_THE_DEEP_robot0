package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ShouldVibrateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class IntakeRetractSPECIFICSampleCommand extends SequentialCommandGroup {
    public IntakeRetractSPECIFICSampleCommand(){
        super(
                new ShouldVibrateCommand(),
                new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                new SetJoystickConstantCommand(Globals.EXTENDO_JOYSTICK_CONSTANT_UP),
                new SetBrushStateCommand(Brush.BrushState.SPITTING),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new WaitCommand(500),
                new SetBrushStateCommand(Brush.BrushState.IDLE)
        );
    }
}
