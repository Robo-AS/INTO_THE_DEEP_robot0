package org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class OuttakeGoHighRungCommand extends SequentialCommandGroup {
    public OuttakeGoHighRungCommand(){
        super(
                new SetLiftStateCommand(Lift.LiftState.HIGH_RUNG),
                new WaitCommand(100),
                new SetWristStateCommand(Arm.WristState.HIGH_RUNG),
                new SetArmStateCommand(Arm.ArmState.HIGH_RUNG),
                new WaitCommand(500),
                new SetArmStateCommand(Arm.ArmState.HIGH_RUNG_SECURE_POSITION)
        );
    }
}
