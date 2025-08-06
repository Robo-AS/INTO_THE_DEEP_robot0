package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class OutttakePutSampleLowBasketGoBackToIdle extends SequentialCommandGroup {
    public OutttakePutSampleLowBasketGoBackToIdle(){
        super(
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new WaitCommand(200),
                new SetArmStateCommand(Arm.ArmState.INIT),
                new WaitCommand(200),
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new SetWristStateCommand(Arm.WristState.INIT),
                new SetLiftStateCommand(Lift.LiftState.IDLE)

        );
    }
}
