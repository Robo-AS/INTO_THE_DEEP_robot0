package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class OuttakeGoBackToIdleCommand extends SequentialCommandGroup {
    public OuttakeGoBackToIdleCommand(){
        super(

                new SetArmStateCommand(Arm.ArmState.INIT),
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new SetWristStateCommand(Arm.WristState.INIT),
                new WaitCommand(200),
                new SetLiftStateCommand(Lift.LiftState.IDLE)

        );
    }
}
