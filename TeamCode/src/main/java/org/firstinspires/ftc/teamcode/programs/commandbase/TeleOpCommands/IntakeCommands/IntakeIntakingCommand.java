package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;


public class IntakeIntakingCommand extends ParallelCommandGroup {
    public IntakeIntakingCommand(){
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
                                new SetBrushStateCommand(Brush.BrushState.INTAKING)
                        ),
                        new DoesNothingCommand(),
                        () -> Extendo.getInstance().extendoMotor.getCurrentPosition() >= 350
                )
//                new SetBrushAngleCommand(Brush.BrushAngle.DOWN),
//                new SetBrushStateCommand(Brush.BrushState.INTAKING)
        );

    }
}
