package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

public class IntakeRetractSubmersibleAutoCommand extends SequentialCommandGroup {
    public IntakeRetractSubmersibleAutoCommand() {
        super(

                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                new SetBrushStateCommand(Brush.BrushState.SPITTING),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new WaitCommand(500),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new SetBrushStateCommand(Brush.BrushState.OUTTAKING),
                new WaitCommand(200),
                new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
                new WaitCommand(200)

        );
    }
}
