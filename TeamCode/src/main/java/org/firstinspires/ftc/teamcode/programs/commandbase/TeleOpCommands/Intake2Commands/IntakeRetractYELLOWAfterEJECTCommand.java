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

public class IntakeRetractYELLOWAfterEJECTCommand extends SequentialCommandGroup {
    public IntakeRetractYELLOWAfterEJECTCommand(){
        super(
                new NEWIntakeIdleCommand(),
                new WaitCommand(70),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new WaitCommand(50),
                new SetBrushStateCommand(Intake.BrushState.IDLE),
                new WaitUntilCommand(Extendo.getInstance()::canOuttakeSample),
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                new WaitUntilCommand(Intake.getInstance()::canStopOuttakingYELLOWAfterEJECT),
                new SetRollersStateCommand(Intake.RollersState.IDLE),
                new ShouldVibrateCommand(),
                new WaitCommand(100),
                new SetClawStateCommand(Arm.ClawState.CLOSED)

        );
    }
}
