package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetSweeperStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands.GoHangLevel2Position;
import org.firstinspires.ftc.teamcode.programs.commandbase.HangCommands.SetSafetyStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands.TriggerHangCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeIntakingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighBasketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoBackToIdleFromHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighBascketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoHighRungCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OuttakeGoLowBasketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakeGoBackToIdleFromLowBasketCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakePutSampleHighBasketGoBackToIdle;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.OutttakePutSampleLowBasketGoBackToIdle;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.OuttakeCommands.PutSpecimenCommand;
import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
import org.firstinspires.ftc.teamcode.programs.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.programs.subsystems.Hang;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.firstinspires.ftc.teamcode.programs.subsystems.Brush;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.utils.geometry.PoseRR;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "🟦TeleOpBlue🟦", group = "OpModes")
public class TeleOpBlue extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    public GamepadEx gamepadEx;

    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
    public static double contantTerm = 0.6, liniarCoefTerm = 0.7;
    private double loopTime = 0;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);

        Globals.HANGING_LEVEL_2 = false;
        Globals.HANGING_LEVEL_3 = false;
        Globals.TELEOP = true;

        robot.initializeHardware(hardwareMap);
        robot.initializeRobot();
        //robot.mecanumDriveTrain.resetTargetPositions();


        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.YELLOW));


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BLUE));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetDesiredColorCommand(Brush.DesiredSampleColor.BOTH));



        //spitting button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetBrushStateCommand(Brush.BrushState.SPITTING_HUMAN_PLAYER),
                                        new DoesNothingCommand(),
                                        () -> robot.brush.brushState != Brush.BrushState.SPITTING_HUMAN_PLAYER
                                )
                        )
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(
                        () -> CommandScheduler.getInstance().schedule(
                                new SetBrushStateCommand(robot.brush.previousBrushState)
                        )
                );



        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new IntakeIntakingCommand(),
                                                new DoesNothingCommand(),
                                                () -> robot.extendo.extendoState == Extendo.ExtendoState.EXTENDING_MINIMUM

                                        ),
                                        new IntakeIdleCommand(),
                                        () -> robot.brush.brushState == Brush.BrushState.IDLE
                                )
                        )
                );


        Trigger extendoTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8);
        extendoTrigger
                .whenActive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM),
                                        new ConditionalCommand(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new IntakeRetractCommand(),
                                                () -> robot.brush.brushAngle == Brush.BrushAngle.UP
                                        ),
                                        () -> robot.extendo.extendoState == Extendo.ExtendoState.RETRACTING
                                )
                        )
                );




        //Lift commands
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new OuttakeGoHighRungCommand(),
                                        new ConditionalCommand(
                                                new PutSpecimenCommand(),
                                                new ConditionalCommand(
                                                        new OuttakeGoBackToIdleFromHighRungCommand(),
                                                        new DoesNothingCommand(),
                                                        () -> robot.arm.clawState == Arm.ClawState.OPEN
                                                )                                                                                                                                                                                                              ,
                                                () -> robot.arm.clawState == Arm.ClawState.CLOSED
                                        ),
                                        () -> robot.lift.liftState == Lift.LiftState.IDLE || robot.lift.liftState == Lift.LiftState.HIGH_BASKET
                                )
                        )

                );



        Trigger liftTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8);
        liftTrigger
                .whenActive(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new OuttakeGoHighBascketCommand(),
                                        new ConditionalCommand(
                                                new OutttakePutSampleHighBasketGoBackToIdle(),
                                                new OuttakeGoBackToIdleFromHighBasketCommand(),
                                                () -> robot.arm.clawState == Arm.ClawState.CLOSED
                                        ),
                                        () -> robot.lift.liftState == Lift.LiftState.IDLE || robot.lift.liftState == Lift.LiftState.HIGH_RUNG
                                )
                        )
                );


        //balaceala de button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new OuttakeCommand(),
                                        new DoesNothingCommand(),
                                        () -> robot.extendo.extendoState == Extendo.ExtendoState.RETRACTING
                                )
                        )
                );

        //HANG
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)//triunghi
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new GoHangLevel2Position(),
                                        new ConditionalCommand(
                                                new SetLiftStateCommand(Lift.LiftState.IDLE),
                                                new DoesNothingCommand(),
                                                () -> robot.lift.liftState == Lift.LiftState.HIGH_BASKET && robot.extendo.extendoState == Extendo.ExtendoState.HANG
                                        ),
                                        () -> robot.lift.liftState != Lift.LiftState.HIGH_BASKET && robot.extendo.extendoState != Extendo.ExtendoState.HANG
                                )
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.X)//patrat
                        .whenPressed(
                                () -> CommandScheduler.getInstance().schedule(
                                        new ConditionalCommand(
                                                new TriggerHangCommand(),
                                                new ConditionalCommand(
                                                        new SetSafetyStateCommand(Hang.SafetyState.TRIGGERED),
                                                        new DoesNothingCommand(),
                                                        () -> robot.hang.safetyState == Hang.SafetyState.IDLE && Globals.HANGING_LEVEL_2
                                                ),
                                                () -> robot.hang.hangState == Hang.HangState.IDLE && Globals.HANGING_LEVEL_2
                                        )
                                )
                        );





        //LOW BASKET
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(
                                () -> CommandScheduler.getInstance().schedule(
                                        new ConditionalCommand(
                                                new OuttakeGoLowBasketCommand(),
                                                new ConditionalCommand(
                                                        new OutttakePutSampleLowBasketGoBackToIdle(),
                                                        new OutttakeGoBackToIdleFromLowBasketCommand(),
                                                        () -> robot.arm.clawState == Arm.ClawState.CLOSED
                                                ),
                                                () -> robot.lift.liftState == Lift.LiftState.IDLE
                                        )
                                )
                        );

        // OPEN/CLOSE CLAW
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetClawStateCommand(Arm.ClawState.OPEN),
                                        new SetClawStateCommand(Arm.ClawState.CLOSED),
                                        () -> robot.arm.clawState == Arm.ClawState.CLOSED
                                )
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetSweeperStateCommand(Sweeper.SweeperState.OPEN),
                                                new WaitCommand(500),
                                                new SetSweeperStateCommand(Sweeper.SweeperState.CLOSED)
                                        ),

                                        new DoesNothingCommand(),
                                        () -> robot.sweeper.sweeperState == Sweeper.SweeperState.CLOSED
                                )
                        )
                );


        gamepadEx.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new InstantCommand(robot.arm::disablePinpoint)
                        )
                );



    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        robot.loop();
        robot.brush.loopBlue();
        robot.extendo.loop(gamepadEx.getLeftY());
        robot.lift.loop();
        robot.arm.loopTeleOp();

        //PENTRU LEVEL 3 HANG
        if(Globals.HANGING_LEVEL_3) {
            robot.mecanumDriveTrain.updateTargetPositionHang(-gamepadEx.getRightY());
        }

        //applying expo function for mecanum
        exponentialJoystickCoord_X_TURN = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * contantTerm;
        exponentialJoystickCoord_X_FORWARD = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * contantTerm;
        exponentialJoystickCoord_Y = (Math.pow(gamepad1.right_stick_y, 3) + liniarCoefTerm * gamepad1.right_stick_y) * contantTerm;

        double turnSpeed = robot.extendo.extendoState == Extendo.ExtendoState.EXTENDING_MINIMUM ? -exponentialJoystickCoord_X_TURN/Globals.DECREASE_TURN_SPEED_CONSTANT :-exponentialJoystickCoord_X_TURN;
        PoseRR drive = new PoseRR(exponentialJoystickCoord_X_FORWARD, -exponentialJoystickCoord_Y, -turnSpeed);
        robot.mecanumDriveTrain.set(drive, 0);


        if(robot.brush.desiredSampleColor == Brush.DesiredSampleColor.BLUE){
            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        else if(robot.brush.desiredSampleColor == Brush.DesiredSampleColor.YELLOW){
            gamepad1.setLedColor(255, 200, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        else if(robot.brush.desiredSampleColor == Brush.DesiredSampleColor.BOTH){
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if(Globals.shouldVibrate){
            gamepad1.rumble(Globals.rumble1, Globals.rumble2, Globals.durationMs);
            Globals.shouldVibrate = false;
        }




//        telemetry.addData("BrushState:", robot.brush.brushState);
//        telemetry.addData("PreviousBrushState:", robot.brush.previousBrushState);
//        telemetry.addData("DesiredColor", robot.brush.desiredSampleColor);
        telemetry.addData("IntakedColor:", robot.brush.intakedSampleColor);
        telemetry.addData("SampleState:", robot.brush.sampleState);
        telemetry.addData("SENSOR DISABLED?:", robot.brush.getREVState());

//        telemetry.addData("AngleServoPosition", robot.brush.brushAngleServo.getPosition());
//        telemetry.addData("Brush Angle", robot.brush.brushAngle);

        telemetry.addData("Current Position EXTENDO", robot.extendo.extendoMotor.getCurrentPosition());
        telemetry.addData("LAST EXTENDO POS", Globals.lastExtendoPos);

//        telemetry.addData("Target Position", robot.extendo.getTargetPosition());
//        telemetry.addData("Extendo State", robot.extendo.extendoState);
//        telemetry.addData("Joystick Y", gamepadEx.gamepad.left_stick_y);
//        telemetry.addData("Joystick Y MODIFIED", robot.extendo.getExponentialJoystickCoef());
//        telemetry.addData("JoystickConstant", robot.extendo.getJoystickConstant());
//        telemetry.addData("SHOULD VIBRATE", Globals.shouldVibrate);

        telemetry.addData("Current Position LIFT", robot.lift.liftMotor.getCurrentPosition());
        telemetry.addData("LAST LIFT POS", Globals.lastLiftPos);

//        telemetry.addData("Reset Lift?", robot.lift.gerResetLift());
//        telemetry.addData("Target Position LIFT", robot.lift.getTargetPosition());


//        telemetry.addData("PINPOINT HEAD", robot.arm.getPinpointHeading());
//        if(gamepad1.cross)
//            robot.arm.updatePINPOINT();
//        telemetry.addData("PROFILE", robot.arm.getProfile());

//        telemetry.addData("HANG_2", Globals.HANGING_LEVEL_2);
//        telemetry.addData("HANG_3", Globals.HANGING_LEVEL_3);
//        telemetry.addData("Current Position Left", robot.mecanumDriveTrain.getCurrentPotionLeft());
//        telemetry.addData("Current Position Right", robot.mecanumDriveTrain.getCurrentPositionRight());
//        telemetry.addData("Target Positon", robot.mecanumDriveTrain.getTargetPositionLeft());


//        telemetry.addData("Hang State", robot.hang.hangState);
//        telemetry.addData("Safety State", robot.hang.safetyState);

        telemetry.addData("RUNNED BASKET",Globals.RUNNED_AUTO_BASKET);

        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}
