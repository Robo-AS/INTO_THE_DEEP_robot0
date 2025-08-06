package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetRollersStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ShouldVibrateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class NEWIntakeRetractSPECIFICSampleCommand extends SequentialCommandGroup {
    public NEWIntakeRetractSPECIFICSampleCommand(){
        super(
                new NEWIntakeIdleCommand(),
                new InstantCommand(() -> Intake.getInstance().setInitialAxonAngle()),
                new ShouldVibrateCommand(),
                new WaitCommand(100),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                //new SetRollersStateCommand(Intake.RollersState.OUTTAKING),
                //new WaitUntilCommand(Intake.getInstance()::canStopOuttakingALIENCE_SPECIFIC_1),
                //new SetRollersStateCommand(Intake.RollersState.IDLE),
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new WaitCommand(50),
                new SetBrushStateCommand(Intake.BrushState.IDLE)
        );
    }
}
