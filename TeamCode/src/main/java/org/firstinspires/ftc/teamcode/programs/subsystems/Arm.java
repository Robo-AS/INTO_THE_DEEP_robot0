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


    public static double rightServoPos = 0.5, leftServoPos = 0.5, clawServoPos = 0, sampleServoPos = 0;



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
        rightServo.setPosition(0.5);
        leftServo.setPosition(0.5);
        clawServo.setPosition(0);
        sampleServo.setPosition(0);

    }


    public void loop(){
        rightServo.setPosition(rightServoPos);
        leftServo.setPosition(leftServoPos);
        clawServo.setPosition(clawServoPos);
        sampleServo.setPosition(sampleServoPos);
    }


}
