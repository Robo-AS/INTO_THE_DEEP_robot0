package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetRollersStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class IntakeRetractSubmersibleAutoCommand extends SequentialCommandGroup {
    public IntakeRetractSubmersibleAutoCommand() {
        super(

                new SetIntakeAngleCommand(Intake.IntakeAngle.UP),
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new WaitCommand(500),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitCommand(200),
                new SetRollersStateCommand(Intake.RollersState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
                new WaitCommand(200)

        );
    }
}
