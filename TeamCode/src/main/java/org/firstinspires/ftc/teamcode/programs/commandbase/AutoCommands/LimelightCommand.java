package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;

public class LimelightCommand extends SequentialCommandGroup {
    public LimelightCommand(){
        super(
                new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_POSE),
                new WaitUntilCommand(Extendo.getInstance()::limelightPoseFinished),
                new WaitCommand(2000),

                new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                new WaitCommand(100),
                new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_RETRACT_POSE).interruptOn(Intake.getInstance()::isSampleDigital),
                new WaitUntilCommand(Extendo.getInstance()::limelightRetractPoseFinished).interruptOn(Intake.getInstance()::isSampleDigital),
                new SetExtendoStateCommand(Extendo.ExtendoState.LIMELIGHT_TAKE_POSE).interruptOn(Intake.getInstance()::isSampleDigital),
                new WaitUntilCommand(Extendo.getInstance()::limelightTakePoseFinished).interruptOn(Intake.getInstance()::isSampleDigital),
                new WaitUntilCommand(Intake.getInstance()::isSampleDigital).withTimeout(1000),
                new SetIntakeStateCommand(Intake.IntakeState.IDLE)
        );
    }
}
