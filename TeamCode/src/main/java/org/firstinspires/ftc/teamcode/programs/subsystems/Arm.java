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

import org.firstinspires.ftc.teamcode.programs.opmodes.auto.BasketPaths;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Arm extends SubsystemBase {
    private static Arm instance = null;
    public CachingServo rightServo, leftServo;
    public CachingServo clawServo, wristServo;
    GoBildaPinpointDriver pinpoint;




    public enum ArmState{
        INIT,
        HIGH_BASKET,
        HIGH_RUNG,
        HIGH_RUNG_SECURE_POSITION
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

    public static double OPEN_clawServo = 0.1, CLOSED_clawServo = 0.75;
    public static double INIT_wristServo = 0, HIGH_BASKET_wristServo = 0.36, HIGH_RUNG_wristServo = 0.9, TRANSITION_wristServo = 0.15;
    public static double INIT_rightServo = 0.075, INIT_leftServo = 0.035;
    public int RANGE_ANGLE = 200;


    public double INIT = 0;
    public static double HIGH_BASKET = 250;
    public static double HIGH_RUNG = 280;
    public double HIGH_RUNG_SECURE_POSITION = 310;


    //MOTION PROFILING STUFF
    private final ElapsedTime time = new ElapsedTime();
    MotionProfile profile;
    public static double targetPosition = 0, previousTarget = 0;
    public static double maxVelocity = 100000, maxAcceleration = 4000;

    double sideAngle = 0;
    public double minAngleBASKET = -30, maxAngleBASKET = 30;
    public double minAngleSPECIMEN = -10, maxAngleSPECIMEN = 10;
    public boolean pinpointDisabled = false;

    public boolean pinpointOn = false;



    public static Arm getInstance(){
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }


    public void initializeHardware(final HardwareMap hardwareMap){
        rightServo = new CachingServo(hardwareMap.get(Servo.class, "armRightServo"));
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftServo = new CachingServo(hardwareMap.get(Servo.class, "armLeftServo"));
        leftServo.setDirection(Servo.Direction.REVERSE);

        clawServo = new CachingServo(hardwareMap.get(Servo.class, "clawServo"));
        clawServo.setDirection(Servo.Direction.FORWARD);

        wristServo = new CachingServo(hardwareMap.get(Servo.class, "wristServo"));
        wristServo.setDirection(Servo.Direction.FORWARD);

        if(Globals.TELEOP)
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

    }


    public void initialize() {
        rightServo.setPosition(INIT_rightServo);
        leftServo.setPosition(INIT_leftServo);
        clawServo.setPosition(OPEN_clawServo);
        wristServo.setPosition(INIT_wristServo);
        //pinpoint.resetPosAndIMU();
        sideAngle = 0;
        pinpointOn = false;
    }





    public void loopAuto(){
        if(armState == ArmState.HIGH_BASKET && BasketPaths.getInstance().SCORE_PRELOAD_COMPLETED)
            sideAngle = 30;
        else sideAngle = 0;//0


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


    public void loopTeleOp(){
        if(pinpointOn && !pinpointDisabled) {
            double currentHeading = getPinpointHeading();
            double referenceHeading = (armState == ArmState.HIGH_RUNG) ? 0 : 135;
            sideAngle = currentHeading - referenceHeading + 180;
            sideAngle = ((sideAngle + 180) % 360 + 360) % 360 - 180;
        }
        else if(!pinpointOn && !pinpointDisabled)
            sideAngle = 0;


        if(armState == ArmState.HIGH_RUNG)
            sideAngle = Math.max(minAngleSPECIMEN, Math.min(maxAngleSPECIMEN, sideAngle));
        else if(armState == ArmState.HIGH_BASKET)
            sideAngle = Math.max(minAngleBASKET, Math.min(maxAngleBASKET, sideAngle));

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
            case HIGH_RUNG_SECURE_POSITION:
                targetPosition = HIGH_RUNG_SECURE_POSITION;
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

    public double getPinpointHeading(){
        pinpoint.update();
        return pinpoint.getHeading() * (180/Math.PI);
    }



    public void testInit(){
//        pinpoint.resetPosAndIMU();
//        sideAngle = 0;
//        rightServo.setPosition(positionToAngleRight(ANGLE));
//        leftServo.setPosition(positionToAngleLeft(ANGLE));
        wristServo.setPosition(HIGH_BASKET_wristServo);
    }

    public void testLOOP(){
//        rightServo.setPosition(positionToAngleRight(INIT_rightServo));
//        leftServo.setPosition(positionToAngleLeft(INIT_leftServo));
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

    public void disablePinpoint(){
        pinpointDisabled = true;
    }

    public void startPinpoint(){pinpointOn = true;}
    public void stopPinpoint(){pinpointOn = false;}


}
