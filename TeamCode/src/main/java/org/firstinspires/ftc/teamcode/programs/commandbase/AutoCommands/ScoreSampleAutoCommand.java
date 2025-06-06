package org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetArmStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetWristStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;

public class ScoreSampleAutoCommand extends SequentialCommandGroup {
    public ScoreSampleAutoCommand(){
        super(
                new WaitUntilCommand(Lift.getInstance()::canOpenClaw),
                new WaitCommand(50),
                new SetClawStateCommand(Arm.ClawState.OPEN),
                new WaitCommand(150)//300
//                new SetArmStateCommand(Arm.ArmState.INIT),
//                new SetClawStateCommand(Arm.ClawState.OPEN),
//                new SetWristStateCommand(Arm.WristState.INIT),
//                new SetLiftStateCommand(Lift.LiftState.IDLE)

        );
    }
}
