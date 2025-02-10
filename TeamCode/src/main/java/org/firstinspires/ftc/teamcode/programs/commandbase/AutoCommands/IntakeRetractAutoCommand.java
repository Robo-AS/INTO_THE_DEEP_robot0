package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetJoystickConstantCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

public class IntakeRetractAutoCommand extends SequentialCommandGroup {
    public IntakeRetractAutoCommand() {
        super(
                //new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetBrushAngleCommand(Brush.BrushAngle.UP),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new WaitUntilCommand(Extendo::canOuttakeSample),
                new SetBrushStateCommand(Brush.BrushState.OUTTAKING),
                new WaitCommand(200),
                new SetBrushStateCommand(Brush.BrushState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
                new WaitCommand(200)

        );
    }
}
