package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class OuttakeGoBackToIdleFromHighRungCommand extends SequentialCommandGroup {
    public OuttakeGoBackToIdleFromHighRungCommand(){
        super(
                new SetLiftStateCommand(Lift.LiftState.UP_FOR_IDLE),
                new SetArmStateCommand(Arm.ArmState.INIT),
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new SetWristStateCommand(Arm.WristState.INIT),
                new WaitCommand(350),
                new SetLiftStateCommand(Lift.LiftState.IDLE)

        );
    }
}
