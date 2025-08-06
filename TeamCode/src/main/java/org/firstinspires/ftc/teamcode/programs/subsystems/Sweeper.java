package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Sweeper extends SubsystemBase {
    private static Sweeper instance = null;
    public CachingServo sweeperServo;

    public enum SweeperState{
        CLOSED,
        OPEN
    }

    public SweeperState sweeperState = SweeperState.CLOSED;
    public static double CLOSED = 0.5, OPEN = 1;


    public static Sweeper getInstance(){
        if (instance == null) {
            instance = new Sweeper();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        sweeperServo = new CachingServo(hardwareMap.get(Servo.class, "sweeperServo"));
        sweeperServo.setDirection(Servo.Direction.FORWARD);

    }

    public void initialize() {
        sweeperServo.setPosition(CLOSED);
    }


    public void testInit(){
        sweeperServo.setPosition(CLOSED);
    }

    public void update(SweeperState state){
        sweeperState = state;
        switch (state){
            case OPEN:
                sweeperServo.setPosition(OPEN);
                break;

            case CLOSED:
                sweeperServo.setPosition(CLOSED);
                break;
        }
    }


}
