package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetRollersStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ShouldVibrateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class NEWOuttakeCommand extends SequentialCommandGroup {
    public NEWOuttakeCommand(){
        super(
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingALIENCE_SPECIFIC),
                new ShouldVibrateCommand(),
                new WaitCommand(100),
                new SetRollersStateCommand(Intake.RollersState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED)
        );
    }
}
