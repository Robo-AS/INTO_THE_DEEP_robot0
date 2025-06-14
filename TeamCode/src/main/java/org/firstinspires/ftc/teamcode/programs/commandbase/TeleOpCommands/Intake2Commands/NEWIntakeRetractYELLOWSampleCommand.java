package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetRollersStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ShouldVibrateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class NEWIntakeRetractYELLOWSampleCommand extends SequentialCommandGroup {
    public NEWIntakeRetractYELLOWSampleCommand(){
        super(
                new NEWIntakeIdleCommand(),
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new WaitCommand(100),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStartSpittingYELLOW),
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new WaitUntilCommand(Intake.getInstance()::canStopSpittingYELLOW),
                new SetBrushStateCommand(Intake.BrushState.IDLE),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingYELLOW_1_TELEOP),

                new SetIntakeStateCommand(Intake.IntakeState.IDLE),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),

                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingYELLOW_2_TELEOP),
                new SetRollersStateCommand(Intake.RollersState.IDLE),
                new SetClawStateCommand(Arm.ClawState.CLOSED),
                new ShouldVibrateCommand(),
                new WaitCommand(100)

        );
    }
}
