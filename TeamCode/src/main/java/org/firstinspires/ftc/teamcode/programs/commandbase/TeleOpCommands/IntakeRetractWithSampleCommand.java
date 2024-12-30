package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

import org.firstinspires.ftc.teamcode.tests.ExtendoTests.SetExtendoStateCommandTEST;

public class IntakeRetractWithSampleCommand extends SequentialCommandGroup {
    public IntakeRetractWithSampleCommand(){
        super(
                new BrushIdleCommand(),
                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                new SetBrushStateCommand(Brush.BrushState.SPITTING),
                new WaitCommand(300),
                new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetExtendoStateCommandTEST(Extendo.ExtendoState.RETRACTING),
                new WaitUntilCommand(Extendo::canOuttakeSample),
                new SetBrushStateCommand(Brush.BrushState.OUTTAKING),
                new WaitCommand(500),
                new SetBrushStateCommand(Brush.BrushState.IDLE)
        );
    }
}
