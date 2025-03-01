package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Hang extends SubsystemBase {
    private static Hang instance = null;
    public CachingServo hangServoLeft, servoSafetyLeft;
    public CachingServo hangServoRight, servoSafetyRight;


    public enum HangState{
        IDLE,
        TRIGGERED
    }

    public enum SafetyState{
        IDLE,
        TRIGGERED
    }


    public HangState hangState = HangState.IDLE;
    public SafetyState safetyState = SafetyState.IDLE;

    public static double IDLE = 0.55;
    public static double TRIGGERED = 0.4;
    public static double IDLE_SAFETY = 0.56;//0.54
    public static double TRIGGERED_SAFETY = 0.4;


    public static Hang getInstance(){
        if (instance == null) {
            instance = new Hang();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        hangServoLeft = new CachingServo(hardwareMap.get(Servo.class, "hangServoLeft"));
        hangServoLeft.setDirection(Servo.Direction.FORWARD);

        hangServoRight = new CachingServo(hardwareMap.get(Servo.class, "hangServoRight"));
        hangServoRight.setDirection(Servo.Direction.REVERSE);

        servoSafetyLeft = new CachingServo(hardwareMap.get(Servo.class, "servoSafetyLeft"));
        servoSafetyLeft.setDirection(Servo.Direction.FORWARD);

        servoSafetyRight = new CachingServo(hardwareMap.get(Servo.class, "servoSafetyRight"));
        servoSafetyRight.setDirection(Servo.Direction.REVERSE);

    }

    public void initialize(){
        hangState = HangState.IDLE;
        safetyState = SafetyState.IDLE;
        hangServoLeft.setPosition(IDLE);
        hangServoRight.setPosition(IDLE);

        servoSafetyLeft.setPosition(IDLE_SAFETY);
        servoSafetyRight.setPosition(IDLE_SAFETY);
    }

    public void updateHang(HangState state){
        hangState = state;
        switch (state){
            case IDLE:
                hangServoLeft.setPosition(IDLE);
                hangServoRight.setPosition(IDLE);
                break;
            case TRIGGERED:
                hangServoLeft.setPosition(TRIGGERED);
                hangServoRight.setPosition(TRIGGERED);
                break;
        }
    }

    public void updateSafety(SafetyState state){
        safetyState = state;
        switch (state){
            case IDLE:
                servoSafetyRight.setPosition(IDLE_SAFETY);
                servoSafetyLeft.setPosition(IDLE_SAFETY);
                break;
            case TRIGGERED:
                servoSafetyRight.setPosition(TRIGGERED_SAFETY);
                servoSafetyLeft.setPosition(TRIGGERED_SAFETY);
                break;
        }
    }



    public void testServosLopp(){
        hangServoLeft.setPosition(TRIGGERED);
        hangServoRight.setPosition(TRIGGERED);
    }

    public void testSafetyLopp(){
        servoSafetyLeft.setPosition(TRIGGERED_SAFETY);
        servoSafetyRight.setPosition(TRIGGERED_SAFETY);
    }
}
