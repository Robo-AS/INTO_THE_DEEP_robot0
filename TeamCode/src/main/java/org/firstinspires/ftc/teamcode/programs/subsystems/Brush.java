package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIdleCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushIntakeCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.BrushThrowingCommand;
import org.firstinspires.ftc.teamcode.programs.commandbase.BrushCommands.SetBrushStateCommand;
import org.firstinspires.ftc.teamcode.tests.OptimizedCommandsTEST.BrushIntakeCommandOPTIMIZED;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Brush extends SubsystemBase{
    private static Brush instance = null;

    public CachingDcMotorEx brushMotor;
    //public CachingServo brushUpDownServo;
    public CachingServo brushSampleServo;
    public RevColorSensorV3 colorSensor0;

    public enum BrushState {
        INTAKING,
        THROWING,
        IDLE;
    }

    public enum BrushAngle{
        UP,
        DOWN;
    }

    public enum IntakedSampleColor {
        RED,
        BLUE,
        YELLOW,
        NOTHING;
    }

    public enum SampleState{
        IS,
        ISNOT;
    }

    public enum DesiredSampleColor {
        YELLOW,
        BLUE,
        RED,
        BOTH;

    }


    public BrushAngle brushAngle = BrushAngle.UP;
    public BrushState brushState = BrushState.IDLE;
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

//        brushUpDownServo = new CachingServo(hardwareMap.get(Servo.class, "brushAngleServo"));
//        brushUpDownServo.setDirection(Servo.Direction.FORWARD);

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));
        brushSampleServo.setDirection(Servo.Direction.FORWARD);

        colorSensor0 = hardwareMap.get(RevColorSensorV3.class, "colorSensor0");
    }


    public void initialize() {
        brushAngle = BrushAngle.UP;
        brushState = BrushState.IDLE;
        desiredSampleColor = DesiredSampleColor.BOTH;
        sampleState = SampleState.ISNOT;
        intakedSampleColor = IntakedSampleColor.NOTHING;
    }



    public void loop(){
        if(brushState == BrushState.IDLE){
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
        }

        if(brushState == BrushState.INTAKING && sampleState == SampleState.ISNOT){
            CommandScheduler.getInstance().schedule(new BrushIntakeCommandOPTIMIZED());
        }

        else if(brushState == BrushState.INTAKING && sampleState == SampleState.IS){
            CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            while(intakedSampleColor == IntakedSampleColor.NOTHING){
                updateIntakedSampleColor();
            }

            if(isRightSampleColorTeleOpBlue()){
                CommandScheduler.getInstance().schedule(new BrushIdleCommand());
            }
            else{
                CommandScheduler.getInstance().schedule(new BrushThrowingCommand());
            }
        }

        if(brushState == BrushState.THROWING && sampleState == SampleState.IS){
            updateSampleState();

            //In case it intakes consecutive sample (one bad and then one right)
            updateIntakedSampleColor();
            if(isRightSampleColorTeleOpBlue())
                CommandScheduler.getInstance().schedule(new BrushIdleCommand());
        }


        if(brushState == BrushState.THROWING && sampleState == SampleState.ISNOT){
            CommandScheduler.getInstance().schedule(new SetBrushStateCommand(BrushState.INTAKING));
        }

    }




    public void updateBrushState(BrushState state){
        brushState = state;
    }

    public void updateDesiredSampleColor(DesiredSampleColor color){
        desiredSampleColor = color;
    }



    public void updateIntakedSampleColor(){
        int red = colorSensor0.red();
        int blue = colorSensor0.blue();

        if(blue > 400)
            intakedSampleColor = IntakedSampleColor.BLUE;
        else if(red > 400 && red < 600)
            intakedSampleColor = IntakedSampleColor.RED;
        else if(red > 600)
            intakedSampleColor = IntakedSampleColor.YELLOW;
        else intakedSampleColor = IntakedSampleColor.NOTHING;
    }




    public void updateSampleState(){
        double distance = colorSensor0.getDistance(DistanceUnit.CM);
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



}
