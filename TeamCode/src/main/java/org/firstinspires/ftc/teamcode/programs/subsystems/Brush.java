package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeRetractSPECIFICSampleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.IntakeCommands.IntakeRetractYELLOWSampleCommand;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Brush extends SubsystemBase{
    private static Brush instance = null;

    public CachingDcMotorEx brushMotor;
    public CachingServo brushAngleServo;
    public CachingServo brushSampleServo;
    public RevColorSensorV3 colorSensor;

    public enum BrushState {
        INTAKING,
        THROWING,
        SPITTING,
        SPITTING_HUMAN_PLAYER,
        OUTTAKING,
        IDLE
    }

    public enum BrushAngle{
        UP,
        DOWN
    }

    public enum IntakedSampleColor {
        RED,
        BLUE,
        YELLOW,
        NOTHING
    }

    public enum SampleState{
        IS,
        ISNOT
    }

    public enum DesiredSampleColor {
        YELLOW,
        BLUE,
        RED,
        BOTH
    }


    public BrushAngle brushAngle = BrushAngle.UP;
    public BrushState brushState = BrushState.IDLE;
    public BrushState previousBrushState;
    public DesiredSampleColor desiredSampleColor = DesiredSampleColor.BOTH;
    public SampleState sampleState = SampleState.ISNOT;
    public IntakedSampleColor intakedSampleColor = IntakedSampleColor.NOTHING;


    public static Brush getInstance() {
        if (instance == null) {
            instance = new Brush();
        }
        return instance;
    }



    public void initializeHardware(final HardwareMap hardwareMap){
        brushMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "brushMotor"));
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brushMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        brushAngleServo = new CachingServo(hardwareMap.get(Servo.class, "brushAngleServo"));
        brushAngleServo.setDirection(Servo.Direction.FORWARD);

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));
        brushSampleServo.setDirection(Servo.Direction.FORWARD);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }


    public void initialize() {
        brushAngle = BrushAngle.UP;
        brushState = BrushState.IDLE;
        desiredSampleColor = DesiredSampleColor.BOTH;
        sampleState = SampleState.ISNOT;
        intakedSampleColor = IntakedSampleColor.NOTHING;
        brushAngleServo.setPosition(Globals.BRUSH_POSITION_UP);
    }



    public void loopBlue(){

        if(brushAngle == BrushAngle.DOWN){
            updateSampleState();
            updateIntakedSampleColor();
        }


        switch (brushState) {
            case SPITTING_HUMAN_PLAYER:
                CommandScheduler.getInstance().schedule(new BrushCommand(-1, 0));
                break;

            case SPITTING:
                CommandScheduler.getInstance().schedule(new BrushCommand(-1, 0.5));
                break;

            case OUTTAKING:
                CommandScheduler.getInstance().schedule(new BrushCommand(0, 1));
                break;

            case IDLE:
                CommandScheduler.getInstance().schedule(new BrushCommand(0, 0.5));
                break;

            case INTAKING:
                if (sampleState == SampleState.ISNOT) {
                    CommandScheduler.getInstance().schedule(
                            new BrushCommand(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_INTAKING)
                    );
                }
                else if (sampleState == SampleState.IS) {
                    CommandScheduler.getInstance().schedule(new BrushCommand(0, 0.5)); // Idle

                    if (intakedSampleColor == IntakedSampleColor.NOTHING) {
                        updateIntakedSampleColor();
                        return;
                    }

                    if (isRightSampleColorTeleOpBlue()) {
                        if (intakedSampleColor == IntakedSampleColor.YELLOW) {
                            CommandScheduler.getInstance().schedule(new IntakeRetractYELLOWSampleCommand());
                        } else {
                            CommandScheduler.getInstance().schedule(new IntakeRetractSPECIFICSampleCommand());
                        }
                    } else {
                        CommandScheduler.getInstance().schedule(new SetBrushStateCommand(BrushState.THROWING));
                    }
                }
                break;

            case THROWING:
                if (sampleState == SampleState.ISNOT) {
                    CommandScheduler.getInstance().schedule(new SetBrushStateCommand(BrushState.INTAKING));
                } else if (sampleState == SampleState.IS) {
                    CommandScheduler.getInstance().schedule(new BrushCommand(Globals.BRUSH_MOTOR_SPEED, 1));
                }
                break;
        }
    }


    public void loopRed(){
        if(brushAngle == BrushAngle.DOWN){
            updateSampleState();
            updateIntakedSampleColor();
        }


        switch (brushState) {
            case SPITTING_HUMAN_PLAYER:
                CommandScheduler.getInstance().schedule(new BrushCommand(-1, 0));
                break;

            case SPITTING:
                CommandScheduler.getInstance().schedule(new BrushCommand(-1, 0.5));
                break;

            case OUTTAKING:
                CommandScheduler.getInstance().schedule(new BrushCommand(0, 1));
                break;

            case IDLE:
                CommandScheduler.getInstance().schedule(new BrushCommand(0, 0.5));
                break;

            case INTAKING:
                if (sampleState == SampleState.ISNOT) {
                    CommandScheduler.getInstance().schedule(
                            new BrushCommand(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_INTAKING)
                    );
                } else if (sampleState == SampleState.IS) {
                    CommandScheduler.getInstance().schedule(new BrushCommand(0, 0.5)); // Idle

                    if (intakedSampleColor == IntakedSampleColor.NOTHING) {
                        updateIntakedSampleColor();
                        return;
                    }


                    if (isRightSampleColorTeleOpRed()) {
                        if (intakedSampleColor == IntakedSampleColor.YELLOW) {
                            CommandScheduler.getInstance().schedule(new IntakeRetractYELLOWSampleCommand());
                        } else {
                            CommandScheduler.getInstance().schedule(new IntakeRetractSPECIFICSampleCommand());
                        }
                    } else {
                        CommandScheduler.getInstance().schedule(new SetBrushStateCommand(BrushState.THROWING));
                    }
                }
                break;

            case THROWING:
                if (sampleState == SampleState.ISNOT) {
                    CommandScheduler.getInstance().schedule(new SetBrushStateCommand(BrushState.INTAKING));
                } else if (sampleState == SampleState.IS) {
                    CommandScheduler.getInstance().schedule(new BrushCommand(Globals.BRUSH_MOTOR_SPEED, 1));
                }
                break;
        }
    }

    public void updateState(BrushState state){
        previousBrushState = brushState;
        brushState = state;
    }

    public void updateDesiredSampleColor(DesiredSampleColor color){
        desiredSampleColor = color;
    }


    public void updateIntakedSampleColor(){
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        if(green > 700)
            intakedSampleColor = IntakedSampleColor.YELLOW;
        else if(blue > 300 && green < 700)
            intakedSampleColor = IntakedSampleColor.BLUE;
        else if(red > 300 && green < 700)
            intakedSampleColor = IntakedSampleColor.RED;
        else intakedSampleColor = IntakedSampleColor.NOTHING;

    }


    public void updateSampleState(){
        double distance = colorSensor.getDistance(DistanceUnit.CM);
        if(distance < 3){
            sampleState = SampleState.IS;
        }
        else sampleState = SampleState.ISNOT;
    }


    public boolean isRightSampleColorTeleOpBlue() {
        switch (desiredSampleColor) {
            case BOTH:
                return ((intakedSampleColor == IntakedSampleColor.BLUE) || (intakedSampleColor == IntakedSampleColor.YELLOW));
            case BLUE:
                return intakedSampleColor == IntakedSampleColor.BLUE;
            case YELLOW:
                return intakedSampleColor == IntakedSampleColor.YELLOW;
            default:
                return false;
        }
    }

    public boolean isRightSampleColorTeleOpRed() {
        switch (desiredSampleColor) {
            case BOTH:
                return ((intakedSampleColor == IntakedSampleColor.RED) || (intakedSampleColor == IntakedSampleColor.YELLOW));
            case RED:
                return intakedSampleColor == IntakedSampleColor.RED;
            case YELLOW:
                return intakedSampleColor == IntakedSampleColor.YELLOW;
            default:
                return false;
        }
    }


    public void updateAngle(BrushAngle angle){
        brushAngle = angle;
        switch (brushAngle){
            case UP:
                brushAngleServo.setPosition(Globals.BRUSH_POSITION_UP);
                break;
            case DOWN:
                brushAngleServo.setPosition(Globals.BRUSH_POSITION_DOWN);
                break;

        }
    }


//    public void updatePreviousBrushState(BrushState state){
//        previousBrushState = state;
//    }

    public void updateShouldVibrate(){
        Globals.shouldVibrate = true;
    }


    public double getCurrent(){
        CurrentUnit unit = CurrentUnit.AMPS;
        return brushMotor.getCurrent(unit);
    }



    public void loopAuto(){
        switch (brushState) {
            case SPITTING_HUMAN_PLAYER:
                CommandScheduler.getInstance().schedule(new BrushCommand(-1, 0));
                break;

            case SPITTING:
                CommandScheduler.getInstance().schedule(new BrushCommand(-1, 0.5));
                break;

            case OUTTAKING:
                CommandScheduler.getInstance().schedule(new BrushCommand(0, 1));
                break;

            case IDLE:
                CommandScheduler.getInstance().schedule(new BrushCommand(0, 0.5));
                break;

            case INTAKING:
                CommandScheduler.getInstance().schedule(new BrushCommand(Globals.BRUSH_MOTOR_SPEED, Globals.BRUSH_SAMPLE_SERVO_SPEED_INTAKING));
                break;


        }
    }

    public void updateSampleStateAuto(double distance){
        if(distance < 3){
            sampleState = SampleState.IS;
        }
        else sampleState = SampleState.ISNOT;
    }

    public boolean isSample(){
        double distance = colorSensor.getDistance(DistanceUnit.CM);
        updateSampleStateAuto(distance);
        return distance < 3;
    }

}
