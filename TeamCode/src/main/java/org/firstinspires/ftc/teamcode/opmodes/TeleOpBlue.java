package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.Globals;
import org.firstinspires.ftc.teamcode.programs.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Brush;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpBlue")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = new Robot();
    private GamepadEx gamepadEx;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gamepadEx = new GamepadEx(gamepad1);

        robot.initializeRobotHadrware();

        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.YELLOW));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BLUE));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BOTH));

        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetBrushStateCommand(Brush.BrushState.INTAKING),
                                        new SetBrushStateCommand(Brush.BrushState.IDLE),
                                        () -> robot.intake.brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();




//        if(robot.intake.brush.brushState == Brush.BrushState.INTAKING && robot.intake.brush.sampleState == Brush.SampleState.ISNOT){
//            CommandScheduler.getInstance().schedule(new BrushCommand(robot.intake.brush, Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED));
//        }
//
//        else if(robot.intake.brush.brushState == Brush.BrushState.INTAKING && robot.intake.brush.sampleState == Brush.SampleState.IS){
//            CommandScheduler.getInstance().schedule(new BrushCommand(robot.intake.brush, 0, 0.5));
//            if(robot.intake.brush.isRightSampleColorBlue(robot.intake.brush.getDesiredSampleColor(), robot.intake.brush.getIntakedSampleColor())){
//                CommandScheduler.getInstance().schedule(new BrushCommand(robot.intake.brush, 0, 0.5));
//            }
//        }


    }
}
