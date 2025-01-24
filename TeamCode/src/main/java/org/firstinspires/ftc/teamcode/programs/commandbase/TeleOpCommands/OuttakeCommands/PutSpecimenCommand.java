package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;

public class PutSpecimenCommand extends SequentialCommandGroup {
    public PutSpecimenCommand(){
        super(
                new SetArmStateCommand(Arm.ArmState.PUT_SPECIMEN),
                new WaitCommand(200),
                new SetClawStateCommand(Arm.ClawState.OPEN)
        );
    }
}
