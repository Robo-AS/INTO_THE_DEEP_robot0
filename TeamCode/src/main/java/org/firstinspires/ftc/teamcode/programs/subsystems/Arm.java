package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.programs.util.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Arm extends SubsystemBase {
    private static Arm instance = null;
    public CachingServo rightServo, leftServo;
    public CachingServo clawServo, sampleServo;

    public enum ArmState{
        INIT,
        HIGH_BASKET,
        HIGH_RUNG,
        PUT_SPECIMEN,
        ARM_ANGLE_TEST
    }

    public ArmState armState = ArmState.INIT;
    public static double rightServoPos = 0.09, leftServoPos = 0.11, clawServoPos = 0, sampleServoPos = 0;
    public static double INIT_rightServo = 0.1, INIT_leftServo = 0.12;
    public static int RANGE_ANGLE = 200;

// pentru servoSample ai la HIGH_BASKET 0.24 pos

    public static double HIGH_BASKET = 220;
    double positionRIGHT, positionLEFT;

    public static Arm getInstance(){
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        rightServo = new CachingServo(hardwareMap.get(Servo.class, "rightServo"));
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftServo = new CachingServo(hardwareMap.get(Servo.class, "leftServo"));
        leftServo.setDirection(Servo.Direction.REVERSE);

        clawServo = new CachingServo(hardwareMap.get(Servo.class, "clawServo"));
        clawServo.setDirection(Servo.Direction.FORWARD);

        sampleServo = new CachingServo(hardwareMap.get(Servo.class, "sampleServo"));
        sampleServo.setDirection(Servo.Direction.FORWARD);

    }


    public void initialize() {
        rightServo.setPosition(INIT_rightServo);
        leftServo.setPosition(INIT_leftServo);
        clawServo.setPosition(0);
        sampleServo.setPosition(0);

    }


    public void loop(){
        positionRIGHT = (HIGH_BASKET/2)/RANGE_ANGLE + INIT_rightServo;
        positionLEFT = (HIGH_BASKET/2)/RANGE_ANGLE + INIT_leftServo;

        rightServo.setPosition(positionRIGHT);
        leftServo.setPosition(positionLEFT);
        sampleServo.setPosition(sampleServoPos);
        clawServo.setPosition(clawServoPos);


    }

//    public void udpdate(ArmState state){
//        armState = state;
//        switch (state){
//            case INIT:
//                rightServo.setPosition(INIT_rightServo);
//                leftServo.setPosition(INIT_leftServo);
//                break;
//
//            case HIGH_BASKET:
//                rightServo.setPosition(degreesToPositonRight(HIGH_BASKET_rightServo));
//                leftServo.setPosition(degreesToPositonLeft(HIGH_BASKET_leftServo));
//                break;
//
//            case HIGH_RUNG:
//                rightServo.setPosition(HIGH_RUNG_rightServo);
//                leftServo.setPosition(HIGH_RUNG_leftServo);
//                break;
//
//            case PUT_SPECIMEN:
//                rightServo.setPosition(PUT_SPECIMEN_rightServo);
//                leftServo.setPosition(PUT_SPECIMEN_leftServo);
//                break;
//
//            case ARM_ANGLE_TEST:
//                rightServo.setPosition(ARM_ANGLE_TEST_rightServo);
//                leftServo.setPosition(ARM_ANGLE_TEST_leftServo);
//                break;
//        }
//
//    }






}
