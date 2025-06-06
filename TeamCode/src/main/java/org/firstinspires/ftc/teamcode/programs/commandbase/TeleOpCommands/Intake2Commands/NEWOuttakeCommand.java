package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ShouldVibrateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class NEWOuttakeCommand extends SequentialCommandGroup {
    public NEWOuttakeCommand(){
        super(
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new SetIntakeStateCommand(Intake.IntakeState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingALIENCE_SPECIFIC_2),
                new ShouldVibrateCommand(),
                new WaitCommand(100),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED)
        );
    }
}
