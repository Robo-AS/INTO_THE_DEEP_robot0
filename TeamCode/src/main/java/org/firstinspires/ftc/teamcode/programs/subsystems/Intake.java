package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.IntakeBlockedSamplesCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.IntakeRetractSPECIFICAfterEJECTCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.IntakeRetractYELLOWAfterEJECTCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.IntakeThrowingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeRetractSPECIFICSampleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.TeleOpCommands.Intake2Commands.NEWIntakeRetractYELLOWSampleCommand;
import org.firstinspires.ftc.teamcode.programs.opmodes.auto.BasketPaths;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }


    public CachingDcMotorEx brushMotor;
    public CachingServo angleServo;
    public CachingServo rollersServo;
    public DigitalChannel pin0;
    public DigitalChannel pin1;
    public DigitalChannel proximitySensor;
    public AnalogInput analogInput;

    private final ElapsedTime currentSpikeTimer = new ElapsedTime();


    public enum IntakeState{
        IDLE, //IDLE + IDLE
        INTAKING, //IN + IN
        SPITTING_HUMAN_PLAYER, //OUT + OUT
        THROWING, //OUT + OUT
    }

    public enum BrushState{
        SPITTING,
        IDLE
    }
    public enum RollersState{
        OUTTAKING,
        IDLE
    }

    public enum IntakeAngle{
        UP,
        DOWN,
        DOWN_AUTO_SPECIMEN
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

    public enum SpecimenBlocked{
        BLOCKED,
        NOT_BLOCKED
    }

    public static double UP_ANGLE = 0.37;
    public static double DOWN_ANGLE = 0.95;
    public static double DOWN_AUTO_SPECIMEN_ANGLE = 0.2;
    public int rotations = 0;

    public int currentAxonAngle, previousAxonAngle, initialAxonAngle;
    public int totalAxonAngle;
    public boolean firstRead = true;
    public boolean sampleThrowed = false;


    public IntakeState intakeState = IntakeState.IDLE;
    public IntakeState previousIntakeState;
    public BrushState brushState;
    public RollersState rollersState;
    public IntakeAngle intakeAngle = IntakeAngle.UP;
    public DesiredSampleColor desiredSampleColor = DesiredSampleColor.BOTH;
    public SampleState sampleState = SampleState.ISNOT;
    public static IntakedSampleColor intakedSampleColor = IntakedSampleColor.NOTHING;
    public SpecimenBlocked specimenBlocked = SpecimenBlocked.NOT_BLOCKED;



    public void initializeHardware(final HardwareMap hardwareMap){
        brushMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "brushMotor"));
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brushMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angleServo = new CachingServo(hardwareMap.get(Servo.class, "angleServo"));
        angleServo.setDirection(Servo.Direction.FORWARD);
        rollersServo = new CachingServo(hardwareMap.get(Servo.class, "rollersServo"));
        rollersServo.setDirection(Servo.Direction.REVERSE);

        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
        proximitySensor = hardwareMap.get(DigitalChannel.class, "paul");
        analogInput = hardwareMap.get(AnalogInput.class, "axonAnalogPin");

    }

    public void initialize(){
        intakeAngle = IntakeAngle.UP;
        intakeState = IntakeState.IDLE;
        desiredSampleColor = DesiredSampleColor.BOTH;
        sampleState = SampleState.ISNOT;
        intakedSampleColor = IntakedSampleColor.NOTHING;
        angleServo.setPosition(UP_ANGLE);
        currentSpikeTimer.reset();

        totalAxonAngle = 0;
        rotations = 0;


    }





    public void updateSampleColor(){
         boolean p0 = pin0.getState();
         boolean p1 = pin1.getState();

         if(p0 && p1)
             intakedSampleColor = IntakedSampleColor.YELLOW;
         else if(p0)
             intakedSampleColor = IntakedSampleColor.BLUE;
         else if(p1)
             intakedSampleColor = IntakedSampleColor.RED;
         else intakedSampleColor = IntakedSampleColor.NOTHING;
    }

    public void updateSampleStateDigital(){
        if(!proximitySensor.getState()){
            sampleState = SampleState.IS;
        }
        else sampleState = SampleState.ISNOT;
    }

    public void updateDesiredSampleColor(DesiredSampleColor color){
        desiredSampleColor = color;
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



    public void updateAngle(IntakeAngle angle){
        intakeAngle = angle;
        switch (intakeAngle){
            case UP:
                angleServo.setPosition(UP_ANGLE);
                break;
            case DOWN:
                angleServo.setPosition(DOWN_ANGLE);
                break;
            case DOWN_AUTO_SPECIMEN:
                angleServo.setPosition(DOWN_AUTO_SPECIMEN_ANGLE);
                break;
        }
    }

    public void updateShouldVibrate(){
        Globals.shouldVibrate = true;
    }

    public void updateIntakeState(IntakeState state){
        previousIntakeState = intakeState;
        intakeState = state;
        switch (intakeState){
            case IDLE:
                brushMotor.setPower(0);
                rollersServo.setPosition(0.5);
                break;
            case INTAKING:
            case THROWING:
                brushMotor.setPower(1);
                rollersServo.setPosition(1);
                break;
            case SPITTING_HUMAN_PLAYER:
                brushMotor.setPower(-1);
                rollersServo.setPosition(0);
                break;
        }
    }

    public void updateBrushState(BrushState state){
        brushState = state;
        switch (brushState){
            case SPITTING:
                brushMotor.setPower(-1);
                break;
            case IDLE:
                brushMotor.setPower(0);
                break;
        }
    }

    public void updateRollersState(RollersState state){
        rollersState = state;
        switch (rollersState){
            case OUTTAKING:
                rollersServo.setPosition(1);
                break;
            case IDLE:
                rollersServo.setPosition(0.5);
                break;
        }
    }

    public void loopAuto(){
        if(intakeAngle == IntakeAngle.DOWN && BasketPaths.getInstance().SCORE_3_COMPLETED){
//            updateSampleColor();
//            updateSampleStateDigital();

            if(brushMotor.getCurrent(CurrentUnit.AMPS) > 2.75 && currentSpikeTimer.seconds() > 1){
                CommandScheduler.getInstance().schedule(new IntakeBlockedSamplesCommand());
                currentSpikeTimer.reset();
            }
        }

        currentAxonAngle = (int) (analogInput.getVoltage() / 3.3 * 360);
        if (firstRead) {
            previousAxonAngle = currentAxonAngle;
            firstRead = false;
            return;
        }

        int delta = currentAxonAngle - previousAxonAngle;

        if(delta > 180)
            rotations --;
        else if (delta < -180)
            rotations ++;

        previousAxonAngle = currentAxonAngle;
        totalAxonAngle = (rotations * 360) + (currentAxonAngle);


    }



    public void loopBlue(){
        currentAxonAngle = (int) (analogInput.getVoltage() / 3.3 * 360);
        if (firstRead) {
            previousAxonAngle = currentAxonAngle;
            firstRead = false;
            return;
        }

        int delta = currentAxonAngle - previousAxonAngle;

        if(delta > 180)
            rotations --;
        else if (delta < -180)
            rotations ++;

        previousAxonAngle = currentAxonAngle;
        totalAxonAngle = (rotations * 360) + (currentAxonAngle);



        if(intakeState == IntakeState.INTAKING){
            updateSampleStateDigital();
            updateSampleColor();

            if(brushMotor.getCurrent(CurrentUnit.AMPS) > 2.75 && currentSpikeTimer.seconds() > 1){
                CommandScheduler.getInstance().schedule(new IntakeBlockedSamplesCommand());
                currentSpikeTimer.reset();
            }

            if(sampleState == SampleState.IS){
                updateSampleColor();
                rollersServo.setPosition(0.5);
                brushMotor.setPower(0);



                if(sampleThrowed){
                    updateSampleColor();
                }

                if (intakedSampleColor == IntakedSampleColor.NOTHING) {
                    updateSampleColor();
                    return;
                }

                if (isRightSampleColorTeleOpBlue()) {
                    //setInitialAxonAngle();
                    if(sampleThrowed){
                        if (intakedSampleColor == IntakedSampleColor.YELLOW)
                            CommandScheduler.getInstance().schedule(new IntakeRetractYELLOWAfterEJECTCommand());
                        else CommandScheduler.getInstance().schedule(new IntakeRetractSPECIFICAfterEJECTCommand());
                        sampleThrowed = false;
                    }
                    else{
                        if (intakedSampleColor == IntakedSampleColor.YELLOW)
                            CommandScheduler.getInstance().schedule(new NEWIntakeRetractYELLOWSampleCommand());
                        else CommandScheduler.getInstance().schedule(new NEWIntakeRetractSPECIFICSampleCommand());
                    }
                }
                else {
                    CommandScheduler.getInstance().schedule(new IntakeThrowingCommand());
                    sampleThrowed = true;
                }

            }
            else sampleThrowed = false;
        }


    }


    public void loopRed(){
        currentAxonAngle = (int) (analogInput.getVoltage() / 3.3 * 360);
        if (firstRead) {
            previousAxonAngle = currentAxonAngle;
            firstRead = false;
            return;
        }

        int delta = currentAxonAngle - previousAxonAngle;

        if(delta > 180)
            rotations --;
        else if (delta < -180)
            rotations ++;

        previousAxonAngle = currentAxonAngle;
        totalAxonAngle = (rotations * 360) + (currentAxonAngle);


        if(intakeAngle == IntakeAngle.DOWN){
            updateSampleStateDigital();
            updateSampleColor();
        }

        if(intakeState == IntakeState.INTAKING){
            if(brushMotor.getCurrent(CurrentUnit.AMPS) > 2.75 && currentSpikeTimer.seconds() > 1){
                CommandScheduler.getInstance().schedule(new IntakeBlockedSamplesCommand());
                currentSpikeTimer.reset();
            }

            if(sampleState == SampleState.IS){
                updateSampleColor();
                rollersServo.setPosition(0.5);
                brushMotor.setPower(0);

                if(sampleThrowed){
                    updateSampleColor();
                }

                if (intakedSampleColor == IntakedSampleColor.NOTHING) {
                    updateSampleColor();
                    return;
                }

                if (isRightSampleColorTeleOpRed()) {
                    //setInitialAxonAngle();
                    if(sampleThrowed){
                        if (intakedSampleColor == IntakedSampleColor.YELLOW)
                            CommandScheduler.getInstance().schedule(new IntakeRetractYELLOWAfterEJECTCommand());
                        else CommandScheduler.getInstance().schedule(new IntakeRetractSPECIFICAfterEJECTCommand());
                        sampleThrowed = false;
                    }
                    else{
                        if (intakedSampleColor == IntakedSampleColor.YELLOW)
                            CommandScheduler.getInstance().schedule(new NEWIntakeRetractYELLOWSampleCommand());
                        else CommandScheduler.getInstance().schedule(new NEWIntakeRetractSPECIFICSampleCommand());
                    }
                }
                else {
                    CommandScheduler.getInstance().schedule(new IntakeThrowingCommand());
                    sampleThrowed = true;
                }

            }
            else sampleThrowed = false;
        }


    }

    public boolean canStopOuttakingYELLOW_1_AUTO(){
        return (totalAxonAngle - initialAxonAngle) >= 50;//50
    }
    public boolean canStopOuttakingYELLOW_2_AUTO(){
        return (totalAxonAngle - initialAxonAngle) >= 80;
    }

    public boolean canStopOuttakingYELLOW_1_TELEOP(){
        return (totalAxonAngle - initialAxonAngle) >= 100;
    }

    public boolean canStopOuttakingYELLOW_2_TELEOP(){
        return (totalAxonAngle - initialAxonAngle) >= 50;
    }

    public boolean canStartSpittingYELLOW(){
        return (totalAxonAngle - initialAxonAngle) >= 20;//30
    }
    public boolean canStopSpittingYELLOW(){
        return (totalAxonAngle - initialAxonAngle) >= 50;
    }

    public boolean canStopOuttakingYELLOWAfterEJECT(){
        return (totalAxonAngle - initialAxonAngle) >= 90;
    }

    public boolean canStopOuttakingALIENCE_SPECIFIC(){
        return (totalAxonAngle - initialAxonAngle) >= 150;//160
    }

    public boolean canStopThrowingWrongSample_TELEOP(){
        return (totalAxonAngle - initialAxonAngle) >= 100;
    }

    public boolean canStopThrowingWrongSample_AUTO(){
        return (totalAxonAngle - initialAxonAngle) >= 150;//100
    }

    public boolean canStopOuttakingSPECIMEN_1_AUTO(){
        return (totalAxonAngle - initialAxonAngle) >= 50;
    }

    public boolean canStopOuttakingSPECIMEN_2_AUTO(){
        return (totalAxonAngle - initialAxonAngle) >= 100;
    }



    public boolean canCloseClaw_AUTO(){
        return (totalAxonAngle - initialAxonAngle) >= 10;
    }


    public void setInitialAxonAngle(){
        initialAxonAngle = totalAxonAngle;
    }



    public void isSpecimenBlocked(){
        CurrentUnit currentUnit = CurrentUnit.AMPS;
        if (brushMotor.getCurrent(currentUnit) > 1.5)
            specimenBlocked = SpecimenBlocked.BLOCKED;
        else specimenBlocked = SpecimenBlocked.NOT_BLOCKED;
    }

    public boolean isSampleDigital(){
        updateSampleStateDigital();
        return !proximitySensor.getState();
    }

    public void setSampleState(SampleState state){
        sampleState = state;
    }
    public void setSampleColor(IntakedSampleColor color){
        intakedSampleColor = color;
    }


    public ElapsedTime getCurrentSpikeTimer(){
        return currentSpikeTimer;
    }

    public void resetTime(){
        currentSpikeTimer.reset();
    }









}
