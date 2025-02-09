package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Arm extends SubsystemBase {
    private static Arm instance = null;
    public CachingServo rightServo, leftServo;
    public CachingServo clawServo, wristServo;


    public enum ArmState{
        INIT,
        HIGH_BASKET,
        HIGH_RUNG,
        PUT_SPECIMEN,
    }

    public enum ClawState{
        OPEN,
        CLOSED
    }

    public enum WristState{
        INIT,
        HIGH_BASKET,
        HIGH_RUNG,
        TRANSITION
    }



    public ArmState armState = ArmState.INIT;
    public ClawState clawState = ClawState.OPEN;
    public WristState wristState = WristState.INIT;

    public static double OPEN_clawServo = 0, CLOSED_clawServo = 0.75;
    public static double INIT_wristServo = 0, HIGH_BASKET_wristServo = 0.3, HIGH_RUNG_wristServo = 1, TRANSITION_wristServo = 0.15;
    public static double INIT_rightServo = 0.14, INIT_leftServo = 0.17;
    public static int RANGE_ANGLE = 200;


    public static double INIT = 0;
    public static double HIGH_BASKET = 230;
    public static double HIGH_RUNG = 325;//220
    public static double PUT_SPECIMEN = 280;


    public static double TEST = HIGH_RUNG;
    //MOTION PROFILING STUFF
    private final ElapsedTime time = new ElapsedTime();
    MotionProfile profile;
    public static double targetPosition = 0, previousTarget = 0;
    public static double maxVelocity = 10000, maxAcceleration = 2500;

    double sideAngle = 0;
    public static double minAngle = -30, maxAngle = 30;

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

        wristServo = new CachingServo(hardwareMap.get(Servo.class, "wristServo"));
        wristServo.setDirection(Servo.Direction.FORWARD);


    }


    public void initialize() {
        rightServo.setPosition(INIT_rightServo);
        leftServo.setPosition(INIT_leftServo);
        clawServo.setPosition(OPEN_clawServo);
        wristServo.setPosition(INIT_wristServo);
        sideAngle = 0;
    }

    public void testInit(){
//        pinpoint.resetPosAndIMU();
//        sideAngle = 0;
        rightServo.setPosition(positionToAngleRight(HIGH_RUNG));
        leftServo.setPosition(positionToAngleLeft(HIGH_RUNG));
    }



    public void loop(){
//        if(armState == ArmState.HIGH_RUNG){
//             sideAngle = getPinpointHeading();
//        }
//        else sideAngle = 0;

//        sideAngle = Math.max(minAngle, Math.min(maxAngle, targetPosition));

        if(targetPosition != previousTarget){
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(previousTarget, 0),
                    new MotionState(targetPosition, 0),
                    maxVelocity,
                    maxAcceleration
            );

            time.reset();
            previousTarget = targetPosition;
        }

        //MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
//        double targetMotionProfile = targetState.getX();
        if(profile!= null){
            MotionState targetState = profile.get(time.seconds());
            double targetMotionProfile = targetState.getX();
            rightServo.setPosition(positionToAngleRight(targetMotionProfile));
            leftServo.setPosition(positionToAngleLeft(targetMotionProfile));
        }


    }

    public void testLOOP(){
        rightServo.setPosition(positionToAngleRight(INIT_rightServo));
        leftServo.setPosition(positionToAngleLeft(INIT_leftServo));
        clawServo.setPosition(OPEN_clawServo);
        wristServo.setPosition(INIT_wristServo);

//        double heading = pinpoint.getHeading() * (180/Math.PI);
//        if(heading > 360){
//            pinpoint.resetPosAndIMU();
//        }
//
//        sideAngle = heading;
//
//        sideAngle = Math.max(-30, Math.min(30, targetPosition));
//
//        if(armState == ArmState.HIGH_BASKET){
//            rightServo.setPosition(positionToAngleRight(0));
//            leftServo.setPosition(positionToAngleLeft(0));
//        }
    }






    public void update(ArmState state){
        armState = state;
        switch (state){
            case INIT:
                targetPosition = INIT;
                break;

            case HIGH_BASKET:
                targetPosition = HIGH_BASKET;
                break;

            case HIGH_RUNG:
                targetPosition = HIGH_RUNG;
                break;

            case PUT_SPECIMEN:
                rightServo.setPosition(positionToAngleRight(PUT_SPECIMEN));
                leftServo.setPosition(positionToAngleLeft(PUT_SPECIMEN));
                previousTarget = PUT_SPECIMEN;
                targetPosition = PUT_SPECIMEN;
                profile = null;
                break;

        }
    }

    public void update(ClawState state){
        clawState = state;
        switch (state){
            case OPEN:
                clawServo.setPosition(OPEN_clawServo);
                break;

            case CLOSED:
                clawServo.setPosition(CLOSED_clawServo);
                break;
        }
    }

    public void update(WristState state){
        wristState = state;
        switch (state){
            case INIT:
                wristServo.setPosition(INIT_wristServo);
                break;

            case HIGH_BASKET:
                wristServo.setPosition(HIGH_BASKET_wristServo);
                break;

            case HIGH_RUNG:
                wristServo.setPosition(HIGH_RUNG_wristServo);
                break;

            case TRANSITION:
                wristServo.setPosition(TRANSITION_wristServo);
                break;
        }
    }




    public double positionToAngleRight(double verticalAngle){
        return (verticalAngle/2 - sideAngle)/RANGE_ANGLE + INIT_rightServo;
    }

    public double positionToAngleLeft(double verticalAngle){
        return (verticalAngle/2 + sideAngle)/RANGE_ANGLE + INIT_leftServo;
    }



    public MotionProfile getProfile(){
        return profile;
    }


}
