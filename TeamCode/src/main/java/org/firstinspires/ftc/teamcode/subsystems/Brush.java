package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Brush extends SubsystemBase{
    private static Brush instance;
    private HardwareMap hardwareMap;

    public CachingDcMotorEx brushMotor;
    public CachingServo brushUpDownServo;
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


    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        brushMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "brushMotor"));
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brushMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        brushUpDownServo = new CachingServo(hardwareMap.get(Servo.class, "brushAngleServo"));
        brushUpDownServo.setDirection(Servo.Direction.FORWARD);

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));
        brushSampleServo.setDirection(Servo.Direction.FORWARD);

        colorSensor0 = hardwareMap.get(RevColorSensorV3.class, "colorSensor0");

    }

    public static Brush getInstance() {
        if (instance == null) {
            instance = new Brush();
        }
        return instance;
    }

    public void initialize() {

    }

    public void loop(){

    }


    public void setState(BrushState state){
        brushState = state;
    }

    public void setDesiredSampleColor(DesiredSampleColor color){
        desiredSampleColor = color;
    }

    public DesiredSampleColor getDesiredSampleColor(){
        return desiredSampleColor;
    }

    public IntakedSampleColor getIntakedSampleColor(){
        int red = colorSensor0.red();
        int blue = colorSensor0.blue();
        //int green = colorSensor0.green();

        if(blue > 400)
            return IntakedSampleColor.BLUE;
        else if(red > 400 && red < 600)
                return  IntakedSampleColor.RED;
        else if(red > 600)
                return IntakedSampleColor.YELLOW;
        return IntakedSampleColor.NOTHING;
    }


    public SampleState getSampleState(){
        double distance = colorSensor0.getDistance(DistanceUnit.CM);
        if(distance < 3){
            return SampleState.IS;
        }
        return SampleState.ISNOT;
    }

    public boolean isRightSampleColorBlue(DesiredSampleColor desiredSampleColor, IntakedSampleColor intakedSampleColor){
        if(desiredSampleColor == DesiredSampleColor.BOTH)
            if(intakedSampleColor != IntakedSampleColor.RED)
                return true;
        else if(desiredSampleColor == DesiredSampleColor.BLUE && intakedSampleColor == IntakedSampleColor.BLUE)
            return true;
        else if(desiredSampleColor == DesiredSampleColor.YELLOW && intakedSampleColor == IntakedSampleColor.YELLOW)
            return true;
        return false;

    }


}
