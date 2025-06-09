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

public class IntakeRetractSPECIFICAfterEJECTCommand extends SequentialCommandGroup {
    public IntakeRetractSPECIFICAfterEJECTCommand(){
        super(
                new NEWIntakeIdleCommand(),
                new ShouldVibrateCommand(),
                new WaitCommand(200),
                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                new SetBrushStateCommand(Intake.BrushState.SPITTING),
                new WaitCommand(50),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
        );
    }
}
