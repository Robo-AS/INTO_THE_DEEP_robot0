package org.firstinspires.ftc.teamcode.programs.subsystems;

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
    private static Brush instance = null;
    private boolean enabled;
    private HardwareMap hardwareMap;

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


    public void initializeHardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        brushMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "brushMotor"));
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brushMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        brushUpDownServo = new CachingServo(hardwareMap.get(Servo.class, "brushAngleServo"));
//        brushUpDownServo.setDirection(Servo.Direction.FORWARD);

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));
        brushSampleServo.setDirection(Servo.Direction.FORWARD);

        colorSensor0 = hardwareMap.get(RevColorSensorV3.class, "colorSensor0");

    }

    public static Brush getInstance() {
        if (instance == null) {
            instance = new Brush();
        }
        instance.enabled = true;
        return instance;
    }

    public void initialize() {

    }

    public void loop(){

    }


    public void updateBrushState(BrushState state){
        brushState = state;
    }

    public void updateDesiredSampleColor(DesiredSampleColor color){
        desiredSampleColor = color;
    }

//    public void updateIntakedSampleColor(IntakedSampleColor color){
//        intakedSampleColor = color;
//    }

//    public void updateIsRightSampleColorBlue(){
//
//    }

    public void updateIntakedSampleColor(){
        int red = colorSensor0.red();
        int blue = colorSensor0.blue();
        //int green = colorSensor0.green();

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


    public boolean isRightSampleColorBlue() {
        switch (desiredSampleColor) {
            case BOTH:
                return intakedSampleColor == IntakedSampleColor.BLUE || intakedSampleColor == IntakedSampleColor.YELLOW;
            case BLUE:
                return intakedSampleColor == IntakedSampleColor.BLUE;
            case YELLOW:
                return intakedSampleColor == IntakedSampleColor.YELLOW;
            default:
                return false;
        }
    }



}
