package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends CommandOpMode {
    private Robot robot = new Robot();
    private GamepadEx gamepadEx;


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeRobotHadrware();

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetDesiredColorCommand(robot.intake, Intake.DesiredSampleColor.YELLOW));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetDesiredColorCommand(robot.intake, Intake.DesiredSampleColor.BLUE));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetDesiredColorCommand(robot.intake, Intake.DesiredSampleColor.BOTH));

    }

    @Override
    public void run(){

    }
}
