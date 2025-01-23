package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(){
        super(
                new WaitUntilCommand(Extendo::canOuttakeSample),
                new SetBrushStateCommand(Brush.BrushState.OUTTAKING),
                new WaitCommand(500),
                new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED)
        );
    }
}
