package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class PutSpecimenCommand extends SequentialCommandGroup {
    public PutSpecimenCommand(){
        super(
                new SetLiftStateCommand(Lift.LiftState.PUT_SPECIMEN),
                new WaitCommand(100),//150
                new SetClawStateCommand(Arm.ClawState.OPEN)
        );
    }
}
