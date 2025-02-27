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

import org.firstinspires.ftc.teamcode.programs.util.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Config
public class Extendo extends SubsystemBase {
    private static Extendo instance = null;
    public CachingDcMotorEx extendoMotor;
    private final ElapsedTime time = new ElapsedTime();

    public enum ExtendoState{
        EXTENDING_MINIMUM,
        RETRACTING,
        EXTENDING_MINIMUM_AUTO,
        TAKE_SAMPLE_AUTO,
        TAKE_SAMPLE_AUTO_NEAR_WALL,
        TAKE_SAMPLE_SPECIMEN,
        TAKE_SPECIMEN_AUTO,
        RETRACT_AUTO_FAIL_SAFE,
        HANG,
        TAKE_SAMPLE_SUBMERSIBLE_1,
        TAKE_SAMPLE_SUBMERSIBLE_2
    }

    public ExtendoState extendoState = ExtendoState.RETRACTING;
    public int EXTENDING_MINIMUM = 400;
    public int RETRACTING = -5;
    public int EXTENDING_MINIMUM_AUTO = 400;
    public int TAKE_SAMPLE_AUTO = 730;
    public int TAKE_SAMPLE_AUTO_NEAR_WALL = 1160;

    public int TAKE_SAMPLE_SPECIMEN = 1160;
    public int TAKE_SPECIMEN_AUTO = 730;
    public int RETRACT_AUTO_FAIL_SAFE = 500;
    public int HANG = 950;
    public int TAKE_SAMPLE_SUBMERSIBLE_1 = 900;
    public int TAKE_SAMPLE_SUBMERSIBLE_2 = 900;


    private PIDController extendo_pid;
    public static double p_extendo = 0.007, i_extendo = 0.11, d_extendo = 0.00006;

    public static int targetPosition = 0;
    public static int currentPosition;
    public static int minPosition = 400, maxPosition = 1160;//550, 1600

//    public static double joystickConstant = 30; //15, 50
    public static double exponentialJoystickCoef;
    public static double contantTerm = 0.6, liniarCoefTerm = 0.7;
    private double joystickConstant;

    MotionProfile profile;
    public static int previousTarget = 0;
    public static double maxVelocity = 1000000, maxAcceleration = 10000;
    public static double maxVelocitySubmersible = 1000, maxAccelerationSubmersible = 10000;

    public Extendo(){
        extendo_pid = new PIDController(p_extendo, i_extendo, d_extendo);
    }


    public static Extendo getInstance(){
        if (instance == null) {
            instance = new Extendo();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        extendoMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendoMotor"));
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize() {
        extendo_pid.reset();
        extendoState = ExtendoState.RETRACTING;
        targetPosition = 0;
        previousTarget = 0;
        joystickConstant = Globals.EXTENDO_JOYSTICK_CONSTANT_UP;
    }


    public void loop(double joystickYCoord){
        currentPosition = extendoMotor.getCurrentPosition();


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


        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double targetMotionProfile = targetState.getX();

        double power = extendo_pid.calculate(currentPosition, targetMotionProfile);
        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        extendoMotor.setPower(power);

        if(extendoState == ExtendoState.EXTENDING_MINIMUM)
            updateTargetPosition(joystickYCoord);

        if(extendoState == ExtendoState.HANG)
            updateTargetPositionHang(joystickYCoord);

    }

    public void update(ExtendoState state){
        extendoState = state;
        switch (extendoState){
            case EXTENDING_MINIMUM:
                targetPosition = EXTENDING_MINIMUM;
                break;
            case RETRACTING:
                targetPosition = RETRACTING;
                break;
            case EXTENDING_MINIMUM_AUTO:
                targetPosition = EXTENDING_MINIMUM_AUTO;
                break;
            case TAKE_SAMPLE_AUTO:
                targetPosition = TAKE_SAMPLE_AUTO;
                break;
            case TAKE_SAMPLE_AUTO_NEAR_WALL:
                targetPosition = TAKE_SAMPLE_AUTO_NEAR_WALL;
                break;
            case TAKE_SAMPLE_SPECIMEN:
                targetPosition = TAKE_SAMPLE_SPECIMEN;
                break;
            case TAKE_SPECIMEN_AUTO:
                targetPosition = TAKE_SPECIMEN_AUTO;
                break;
            case RETRACT_AUTO_FAIL_SAFE:
                targetPosition = RETRACT_AUTO_FAIL_SAFE;
                break;
            case HANG:
                targetPosition = HANG;
                break;
            case TAKE_SAMPLE_SUBMERSIBLE_1:
                targetPosition = TAKE_SAMPLE_SUBMERSIBLE_1;
                break;
            case TAKE_SAMPLE_SUBMERSIBLE_2:
                targetPosition = TAKE_SAMPLE_SUBMERSIBLE_2;
                break;

        }
    }

    public void updateTargetPosition(double joystickYCoord){
        exponentialJoystickCoef = (Math.pow(joystickYCoord, 3) + liniarCoefTerm * joystickYCoord) * contantTerm;

        targetPosition += (int)(exponentialJoystickCoef * joystickConstant);
        targetPosition = Math.max(minPosition, Math.min(maxPosition, targetPosition)); //limita de prosti
    }

    public void updateTargetPositionHang(double joystickYCoord){
        exponentialJoystickCoef = (Math.pow(joystickYCoord, 3) + liniarCoefTerm * joystickYCoord) * contantTerm;

        targetPosition += (int)(exponentialJoystickCoef * joystickConstant);
        targetPosition = Math.max(RETRACTING, Math.min(HANG, targetPosition));
    }



    public void updateJoystickConstant(double constant){ joystickConstant = constant; }

    public int getTargetPosition(){
        return targetPosition;
    }

    public double getJoystickConstant(){return joystickConstant;}


    public static boolean canOuttakeSample(){
        return currentPosition <= 300;
    }


    public void loopAuto(){
        currentPosition = extendoMotor.getCurrentPosition();


        if(targetPosition != previousTarget){
            if(extendoState == ExtendoState.TAKE_SAMPLE_SUBMERSIBLE_1 || extendoState == ExtendoState.TAKE_SAMPLE_SUBMERSIBLE_2){
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(previousTarget, 0),
                        new MotionState(targetPosition, 0),
                        maxVelocitySubmersible,
                        maxAccelerationSubmersible
                );
            }
            else{
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(previousTarget, 0),
                        new MotionState(targetPosition, 0),
                        maxVelocity,
                        maxAcceleration
                );
            }


            time.reset();
            previousTarget = targetPosition;
        }


        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double targetMotionProfile = targetState.getX();

        double power = extendo_pid.calculate(currentPosition, targetMotionProfile);
        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        extendoMotor.setPower(power);
    }


}
