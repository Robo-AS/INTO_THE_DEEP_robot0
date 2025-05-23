package org.firstinspires.ftc.teamcode.programs.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.programs.util.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Lift extends SubsystemBase{
    private static Lift instance = null;
    public CachingDcMotorEx liftMotor, followerMotor;
    private final ElapsedTime time = new ElapsedTime();


    public enum LiftState{
        LOW_BASKET,
        HIGH_BASKET,
        LOW_RUNG,
        HIGH_RUNG,
        PUT_SPECIMEN,
        UP_FOR_IDLE,
        HIGH_BASKET_AUTO,
        IDLE
    }

    public LiftState liftState = LiftState.IDLE;

    public static int HIGH_BASKET = 900;
    public static int HIGH_RUNG = 520;
    public static int LOW_BASKET = 400;
    public static int LOW_RUNG = 0;
    public static int PUT_SPECIMEN = 160;//150
    public static int UP_FOR_IDLE = 300;//300
    public static int HIGH_BASKET_AUTO = 840 ;
    public static int IDLE = 0;



    private PIDController lift_pid;
    public int targetPosition;
    public int currentPosition;

    public static double p_lift = 0.0056, d_lift = 0.00009, i_lift = 0.15;
    public static double p_hang = 0.04, d_hang = 0, i_hang = 0;

    MotionProfile profile;
    public int previousTarget = 0;
    public static double maxVelocityUP = 10000000, maxAccelerationUP = 200000;
    public static double maxVelocityDOWN = 1000000, maxAccelerationDOWN = 4000;
    public static double maxVelocityPUT_SPECIMEN = 10000000, maxAccelerationPUT_SPECIMEN = 60000;
    public static double maxVelocityHANG = 800, maxAccelerationHANG = 3000000;


    public Lift(){
        lift_pid = new PIDController(p_lift, i_lift, d_lift);
    }

    public static Lift getInstance(){
        if (instance == null) {
            instance = new Lift();
        }
        return instance;
    }



    public void initializeHardware(final HardwareMap hardwareMap){
        liftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "liftMotor"));
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        followerMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "followerMotor"));
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //followerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        followerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize(){
        lift_pid.reset();
        liftState = LiftState.IDLE;

        currentPosition = liftMotor.getCurrentPosition();
        targetPosition = currentPosition;
        previousTarget = currentPosition;



        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPosition, 0),
                new MotionState(targetPosition, 0),
                maxVelocityDOWN,
                maxAccelerationDOWN
        );

        time.reset();
    }


    public void loop(){
        currentPosition = liftMotor.getCurrentPosition();
        if(Globals.HANGING_LEVEL_2){
            p_lift = p_hang;
            d_lift = d_hang;
            i_lift = i_hang;
        }
        else{
            p_lift = 0.0056;
            d_lift = 0.00009;
            i_lift = 0.15;
        }

        if(targetPosition != previousTarget) {
            if(targetPosition > previousTarget) { //profile pentru extindere
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(previousTarget, 0),
                        new MotionState(targetPosition, 0),
                        maxVelocityUP,
                        maxAccelerationUP
                );
            }

            if(targetPosition < previousTarget){//profile pentru retragere

                if(liftState == LiftState.PUT_SPECIMEN){
                    profile = MotionProfileGenerator.generateSimpleMotionProfile(
                            new MotionState(previousTarget, 0),
                            new MotionState(targetPosition, 0),
                            maxVelocityPUT_SPECIMEN,
                            maxAccelerationPUT_SPECIMEN
                    );
                }
                else if (Globals.HANGING_LEVEL_2){
                    profile = MotionProfileGenerator.generateSimpleMotionProfile(
                            new MotionState(previousTarget, 0),
                            new MotionState(targetPosition, 0),
                            maxVelocityHANG,
                            maxAccelerationHANG
                    );
                }

                else{
                    profile = MotionProfileGenerator.generateSimpleMotionProfile(
                            new MotionState(previousTarget, 0),
                            new MotionState(targetPosition, 0),
                            maxVelocityDOWN,
                            maxAccelerationDOWN
                    );
                }

            }

            time.reset();
            previousTarget = targetPosition;
        }



        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double targetMotionProfile = targetState.getX();

        lift_pid.setPID(p_lift, i_lift, d_lift);
        double power = lift_pid.calculate(currentPosition, targetMotionProfile);
        liftMotor.setPower(power);
        followerMotor.setPower(power);

    }




    public void update(LiftState state){
        liftState = state;
        switch (liftState){
            case LOW_BASKET:
                targetPosition = LOW_BASKET;
                break;

            case HIGH_BASKET:
                targetPosition = HIGH_BASKET;
                break;

            case LOW_RUNG:
                targetPosition = LOW_RUNG;
                break;

            case HIGH_RUNG:
                targetPosition = HIGH_RUNG;
                break;

            case PUT_SPECIMEN:
                targetPosition = PUT_SPECIMEN;
                break;
            case UP_FOR_IDLE:
                targetPosition = UP_FOR_IDLE;
                break;

            case HIGH_BASKET_AUTO:
                targetPosition = HIGH_BASKET_AUTO;
                break;
            case IDLE:
                targetPosition = IDLE;
                break;
        }

    }

    public int getCurrentPosition(){
        return currentPosition;
    }

    public int getTargetPosition(){
        return targetPosition;
    }

    public boolean canRotateWrist(){
        return currentPosition >= 250;
    }

    public boolean canRotateArmHighBasket(){
        return currentPosition >= 150;
    }


//        public void testLoop(){
//        currentPosition = liftMotor.getCurrentPosition();
//
//        lift_pid.setPID(p_lift, i_lift, d_lift);
//        double pid = lift_pid.calculate(currentPosition, targetPosition);
//        double power = pid;
//        liftMotor.setPower(power);
//        followerMotor.setPower(power);
//    }


    public void hangTestLoop(double gamepadY){
        liftMotor.setPower(gamepadY);
        followerMotor.setPower(gamepadY);
    }

    public double getCurrentLiftMotor(){
        CurrentUnit currentUnit = CurrentUnit.AMPS;
        return liftMotor.getCurrent(currentUnit);
    }


    public double getCurrentFollowerMotor(){
        CurrentUnit currentUnit = CurrentUnit.AMPS;
        return followerMotor.getCurrent(currentUnit);
    }


    public boolean canOpenClaw(){
        return currentPosition > 800;
    }
}
