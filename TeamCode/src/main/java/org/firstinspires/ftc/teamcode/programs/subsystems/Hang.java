package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Hang extends SubsystemBase {
    private static Hang instance = null;
    public CachingServo hangServoLeft;
    public CachingServo hangServoRight;


    public enum HangState{
        IDLE,
        TRIGGERED
    }


    public HangState hangState = HangState.IDLE;

    public static double IDLE = 0.55;
    public static double TRIGGERED = 0.4;


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

    }

    public void initialize(){
        hangState = HangState.IDLE;
        hangServoLeft.setPosition(IDLE);
        hangServoRight.setPosition(IDLE);
    }

    public void update(HangState state){
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



    public void testServosLopp(){
        hangServoLeft.setPosition(TRIGGERED);
        hangServoRight.setPosition(TRIGGERED);
    }
}
