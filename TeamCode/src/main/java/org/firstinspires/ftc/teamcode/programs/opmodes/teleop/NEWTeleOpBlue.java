package org.firstinspires.ftc.teamcode.programs.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.AutoCommands.SpecimenAuto.IntakeRetractSPECIMENAutoCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.DoesNothingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.ExtendoCommands.SetExtendoStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.HangCommands.SetSafetyStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeAngleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.IntakeCommand.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.NEWSetDesiredColorCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.SetSweeperStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands.GoHangLevel2Position;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.AscentCommands.TriggerHangCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeIntakingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeRetractCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWOuttakeCommand;
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
import org.firstinspires.ftc.teamcode.programs.subsystems.Intake;
import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
import org.firstinspires.ftc.teamcode.programs.subsystems.Sweeper;
import org.firstinspires.ftc.teamcode.programs.util.Globals;
import org.firstinspires.ftc.teamcode.programs.util.NEWRobot;
import org.firstinspires.ftc.teamcode.utils.geometry.PoseRR;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "🟦NEWTeleOpBlue🟦", group = "OpModes")
public class NEWTeleOpBlue extends CommandOpMode {
    private final NEWRobot robot = NEWRobot.getInstance();
    public GamepadEx gamepadEx;
    private Follower follower;
    public boolean AUTO_IN_TELEOP = false;
    private final int sensorTimeOut = 1500;

    private final Pose startPose = new Pose(42.077922077922075, 65.6883116883117, Math.toRadians(180));

    public static Pose takeSpecimenPose = new Pose(27.584415584415584, 40.90909090909091, Math.toRadians(-130));
    public static Pose takeSpecimenPoseControlPoint = new Pose(28.753246753246753, 72.46753246753246, Math.toRadians(-130));

    public static Pose score1Pose = new Pose(37.40909090909091, 68.83246753246754, Math.toRadians(180));
    public static Pose score1SMALLPose = new Pose(39.40909090909091, 68.83246753246754, Math.toRadians(180));

    public static Pose score2Pose = new Pose(37.40909090909091, 70.83246753246754, Math.toRadians(180));
    public static Pose score2SMALLPose = new Pose(39.40909090909091, 70.83246753246754, Math.toRadians(180));

    public static Pose score3Pose = new Pose(37.40909090909091, 72.83246753246754, Math.toRadians(180));
    public static Pose score3SMALLPose = new Pose(39.40909090909091, 72.83246753246754, Math.toRadians(180));

    public static Pose score4Pose = new Pose(37.40909090909091, 74.83246753246754, Math.toRadians(180));
    public static Pose score4SMALLPose = new Pose(39.40909090909091, 74.83246753246754, Math.toRadians(180));

    public static Pose score5Pose = new Pose(37.40909090909091, 76.83246753246754, Math.toRadians(180));
    public static Pose score5SMALLPose = new Pose(39.40909090909091, 76.83246753246754, Math.toRadians(180));


    private PathChain take1, score1, score1SMALL;
    private PathChain take2, score2, score2SMALL;
    private PathChain take3, score3, score3SMALL;
    private PathChain take4, score4, score4SMALL;
    private PathChain take5, score5, score5SMALL;


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



        //Choosing sample color button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new NEWSetDesiredColorCommand(Intake.DesiredSampleColor.YELLOW));


        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new NEWSetDesiredColorCommand(Intake.DesiredSampleColor.BLUE));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new NEWSetDesiredColorCommand(Intake.DesiredSampleColor.BOTH));




        //Controling the brush button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new NEWIntakeIntakingCommand(),
                                                new DoesNothingCommand(),
                                                () -> robot.extendo.extendoState == Extendo.ExtendoState.EXTENDING_MINIMUM

                                        ),
                                        new NEWIntakeIdleCommand(),
                                        () -> robot.intake.intakeState == Intake.IntakeState.IDLE
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
                                                new NEWIntakeRetractCommand(),
                                                () -> robot.intake.intakeAngle == Intake.IntakeAngle.UP
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
                                        new SequentialCommandGroup(
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                new WaitCommand(50),
                                                new OuttakeGoHighRungCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING)
                                        ),

                                        new ConditionalCommand(
                                                new OuttakeGoBackToIdleFromHighRungCommand(),
                                                new ConditionalCommand(
                                                        new SetIntakeStateCommand(Intake.IntakeState.SPITTING_HUMAN_PLAYER),
                                                        new ConditionalCommand(
                                                                new PutSpecimenCommand(),
                                                                new DoesNothingCommand(),
                                                                () -> robot.lift.liftState != Lift.LiftState.PUT_SPECIMEN && robot.arm.clawState == Arm.ClawState.CLOSED
                                                        ),
                                                        () -> (robot.lift.liftState == Lift.LiftState.IDLE) && robot.arm.clawState == Arm.ClawState.OPEN
                                                )                                                                                                                                                                                                              ,
                                                () -> (robot.lift.liftState == Lift.LiftState.HIGH_RUNG || robot.lift.liftState == Lift.LiftState.PUT_SPECIMEN) && robot.arm.clawState == Arm.ClawState.OPEN
                                        ),
                                        () -> robot.lift.liftState != Lift.LiftState.HIGH_RUNG && robot.arm.clawState == Arm.ClawState.CLOSED
                                )
                        )
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new SetIntakeStateCommand(robot.intake.previousIntakeState),
                                        new DoesNothingCommand(),
                                        () -> robot.lift.liftState == Lift.LiftState.IDLE && robot.arm.clawState == Arm.ClawState.OPEN
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
                                        () -> robot.lift.liftState != Lift.LiftState.HIGH_BASKET
                                )
                        )
                );


        //balaceala de button logic
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new ConditionalCommand(
                                        new NEWOuttakeCommand(),
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
                                                () -> robot.lift.liftState == Lift.LiftState.HANG && robot.extendo.extendoState == Extendo.ExtendoState.HANG
                                        ),
                                        () -> robot.lift.liftState != Lift.LiftState.HANG && robot.extendo.extendoState != Extendo.ExtendoState.HANG
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





        //LOW BASKET (the X)
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
                                        () -> robot.lift.liftState != Lift.LiftState.LOW_BASKET
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


        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(
                        () -> CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new InstantCommand(this::initializeAUTO_IN_TELEOP)
                                                ),

                                                new SequentialCommandGroup(
                                                        new InstantCommand(() -> follower.breakFollowing()),
//                                                        new InstantCommand(() -> CommandScheduler.getInstance().reset()),
                                                        new InstantCommand(() -> AUTO_IN_TELEOP = false)

                                                ),
                                                ()-> !AUTO_IN_TELEOP
                                        )

                                )
                        )
                );



    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        if(AUTO_IN_TELEOP){
            follower.update();

        }

        robot.loop();
//        robot.intake.loopBlue();
        robot.extendo.loop(gamepadEx.getLeftY());
        robot.lift.loop();
//        robot.arm.loopTeleOp();

        robot.arm.loopAuto();
        robot.intake.loopAuto();


//        PENTRU LEVEL 3 HANG
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


        if(robot.intake.desiredSampleColor == Intake.DesiredSampleColor.BLUE){
            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
        else if(robot.intake.desiredSampleColor == Intake.DesiredSampleColor.YELLOW){
            gamepad1.setLedColor(255, 200, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        else if(robot.intake.desiredSampleColor == Intake.DesiredSampleColor.BOTH){
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if(Globals.shouldVibrate){
            gamepad1.rumble(Globals.rumble1, Globals.rumble2, Globals.durationMs);
            Globals.shouldVibrate = false;
        }

        telemetry.addData("AUTO_IN_TELEOP", AUTO_IN_TELEOP);
        telemetry.addData("SampleColor", robot.intake.intakedSampleColor);
        telemetry.addData("e?", robot.intake.sampleState);



//        telemetry.addData("BrushState:", robot.brush.brushState);
//        telemetry.addData("PreviousBrushState:", robot.brush.previousBrushState);
//        telemetry.addData("DesiredColor", robot.brush.desiredSampleColor);
//        telemetry.addData("IntakedColor:", robot.intake.intakedSampleColor);
//        telemetry.addData("SampleState:", robot.intake.sampleState);
//        telemetry.addData("BrushCurrent", robot.intake.brushMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("CurrentSpikeTimer", robot.intake.getCurrentSpikeTimer().seconds());

//        telemetry.addData("TIME PASSED", robot.intake.getTimePassed());

//        telemetry.addData("SENSOR DISABLED?:", robot.brush.getREVState());

//        telemetry.addData("AngleServoPosition", robot.brush.brushAngleServo.getPosition());
//        telemetry.addData("Brush Angle", robot.brush.brushAngle);

//        telemetry.addData("Current Position EXTENDO", robot.extendo.extendoMotor.getCurrentPosition());
//        telemetry.addData("LAST EXTENDO POS", Globals.lastExtendoPos);

//        telemetry.addData("Target Position", robot.extendo.getTargetPosition());
//        telemetry.addData("Extendo State", robot.extendo.extendoState);
//        telemetry.addData("Joystick Y", gamepadEx.gamepad.left_stick_y);
//        telemetry.addData("Joystick Y MODIFIED", robot.extendo.getExponentialJoystickCoef());
//        telemetry.addData("JoystickConstant", robot.extendo.getJoystickConstant());
//        telemetry.addData("SHOULD VIBRATE", Globals.shouldVibrate);

//        telemetry.addData("Current Position LIFT", robot.lift.liftMotor.getCurrentPosition());
//        telemetry.addData("LAST LIFT POS", Globals.lastLiftPos);

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

//        telemetry.addData("RUNNED BASKET",Globals.RUNNED_AUTO_BASKET);

//        telemetry.addData("CURRENT:", robot.intake.currentAxonAngle);
//        telemetry.addData("PREVIOUS:", robot.intake.previousAxonAngle);
//        telemetry.addData("TOTAL:", robot.intake.totalAxonAngle);
//        telemetry.addData("INITIAL:", robot.intake.initialAxonAngle);
//        telemetry.addData("FIRST READ", robot.intake.firstRead);

//        telemetry.addData("Current Position", robot.extendo.extendoMotor.getCurrentPosition());
//        telemetry.addData("Target Position", robot.extendo.getTargetPosition());
//        telemetry.update();

        double loop = System.nanoTime();
        telemetry.addData("Hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }




    public void initializeAUTO_IN_TELEOP(){

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

         take1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(takeSpecimenPoseControlPoint), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

         score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

         score1SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(score1SMALLPose)))
                .setConstantHeadingInterpolation(score1SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        take2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1SMALLPose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(score1SMALLPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        score2SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(score2SMALLPose)))
                .setConstantHeadingInterpolation(score2SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        take3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2SMALLPose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(score2SMALLPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        score3SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(score3SMALLPose)))
                .setConstantHeadingInterpolation(score3SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        take4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3SMALLPose), new Point(takeSpecimenPose)))
                .setLinearHeadingInterpolation(score3SMALLPose.getHeading(), takeSpecimenPose.getHeading())
                .setZeroPowerAccelerationMultiplier(16)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score4Pose)))
                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(14)
                .build();

        score4SMALL = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose), new Point(score4SMALLPose)))
                .setConstantHeadingInterpolation(score4SMALLPose.getHeading())
                .setZeroPowerAccelerationMultiplier(10)
                .build();

//        take5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score4SMALLPose), new Point(takeSpecimenPose)))
//                .setLinearHeadingInterpolation(score4SMALLPose.getHeading(), takeSpecimenPose.getHeading())
////                .setZeroPowerAccelerationMultiplier(16)
//                .build();
//
//        score5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(takeSpecimenPose), new Point(score5Pose)))
//                .setLinearHeadingInterpolation(takeSpecimenPose.getHeading(), score5Pose.getHeading())
////                .setZeroPowerAccelerationMultiplier(14)
//                .build();
//
//        score5SMALL = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(score5Pose), new Point(score5SMALLPose)))
//                .setConstantHeadingInterpolation(score5SMALLPose.getHeading())
////                .setZeroPowerAccelerationMultiplier(10)
//                .build();






        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> AUTO_IN_TELEOP = true),
                        new WaitCommand(1000),

                        new FollowPath(follower, take1, true, 1)
                                .alongWith(
                                        new SetExtendoStateCommand(Extendo.ExtendoState.EXTENDING_MINIMUM_AUTO)
                                ),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),



                        new FollowPath(follower, score1, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                new WaitCommand(50),
                                                new OuttakeGoHighRungCommand()
                                        )
                                ),
//                        new WaitCommand(250),
                        new FollowPath(follower, score1SMALL, true, 1),

                        new FollowPath(follower, take2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                ),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),


                        new FollowPath(follower, score2, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                new WaitCommand(50),
                                                new OuttakeGoHighRungCommand()
                                        )
                                ),
//                        new WaitCommand(250),
                        new FollowPath(follower, score2SMALL, true, 1),



                        new FollowPath(follower, take3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                ),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),


                        new FollowPath(follower, score3, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                new WaitCommand(50),
                                                new OuttakeGoHighRungCommand()
                                        )
                                ),
//                        new WaitCommand(250),
                        new FollowPath(follower, score3SMALL, true, 1),




                        new FollowPath(follower, take4, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new PutSpecimenCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.RETRACTING),
                                                new OuttakeGoBackToIdleFromHighRungCommand()
                                        )
                                ),
                        new SetIntakeAngleCommand(Intake.IntakeAngle.DOWN),
                        new SetIntakeStateCommand(Intake.IntakeState.INTAKING),
                        new SetExtendoStateCommand(Extendo.ExtendoState.TAKE_SPECIMEN_AUTO),
                        new WaitUntilCommand(robot.intake::isSampleDigital).withTimeout(sensorTimeOut),
                        new SetIntakeStateCommand(Intake.IntakeState.IDLE),


                        new FollowPath(follower, score4, true, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new IntakeRetractSPECIMENAutoCommand(),
                                                new SetExtendoStateCommand(Extendo.ExtendoState.EXTEND_SPECIMEN_EXIT),
                                                new WaitCommand(50),
                                                new OuttakeGoHighRungCommand()
                                        )
                                ),
//                        new WaitCommand(250),
                        new FollowPath(follower, score4SMALL, true, 1),
                        new WaitCommand(200),
                        new PutSpecimenCommand(),
                        new WaitCommand(200),




                        new InstantCommand(() -> AUTO_IN_TELEOP = false),
                        new InstantCommand(() -> follower.breakFollowing()),
                        new InstantCommand(this::reInitializeMecanum)

                )

        );


    }



    public void reInitializeMecanum(){
        robot.mecanumDriveTrain.initializeHardware(hardwareMap);
        robot.mecanumDriveTrain.initialize();
    }
}