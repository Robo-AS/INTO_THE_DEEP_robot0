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
        TAKE_SAMPLE_SUBMERSIBLE_2,
        MAXIMUM,
        LIMELIGHT_POSE,
        LIMELIGHT_RETRACT_POSE,
        LIMELIGHT_TAKE_POSE,
        LIMELIGHT_TAKE_POSE_AFTER_THROWING
    }

    public ExtendoState extendoState = ExtendoState.RETRACTING;
    public static int EXTENDING_MINIMUM = 330;
    public static int RETRACTING = -15;
    public static int HANG = 800;

    public int EXTENDING_MINIMUM_AUTO = 815;
    public int TAKE_SAMPLE_AUTO = 1000;
    public int TAKE_SAMPLE_AUTO_NEAR_WALL = 1160;

    public int TAKE_SAMPLE_SPECIMEN = 1160;
    public int TAKE_SPECIMEN_AUTO = 730;
    public int RETRACT_AUTO_FAIL_SAFE = 500;

    public int TAKE_SAMPLE_SUBMERSIBLE_1 = 900;
    public int TAKE_SAMPLE_SUBMERSIBLE_2 = 900;
    public int MAXIMUM = 1150;


    private PIDController extendo_pid;
    public static double p_extendo = 0.014, i_extendo = 0.009, d_extendo = 0.0005;

    public static int targetPosition = 0;
    public int currentPosition;
    public int minPosition = 330, maxPosition = 1150;

//    public static double joystickConstant = 30; //15, 50
    public double exponentialJoystickCoef;
    public double contantTerm = 0.6, liniarCoefTerm = 0.7;
    private double joystickConstant;

    MotionProfile profile;
    public int previousTarget = 0;
    public static double maxVelocity = 1000000, maxAcceleration = 40000;
    public double maxVelocitySubmersible = 1000, maxAccelerationSubmersible = 10000;

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
        extendoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders(){
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize() {
        extendo_pid.reset();
        extendoState = ExtendoState.RETRACTING;

        currentPosition = extendoMotor.getCurrentPosition();
        targetPosition = currentPosition;
        previousTarget = currentPosition;
        joystickConstant = Globals.EXTENDO_JOYSTICK_CONSTANT_UP;


        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPosition, 0),
                new MotionState(targetPosition, 0),
                maxVelocity,
                maxAcceleration
        );

        time.reset();
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

        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        double power = extendo_pid.calculate(currentPosition, targetMotionProfile);
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
            case MAXIMUM:
                targetPosition = MAXIMUM;
                break;
            case LIMELIGHT_POSE:
                targetPosition = Math.min(Globals.extendoDistance, MAXIMUM);
                break;
            case LIMELIGHT_RETRACT_POSE:
                targetPosition = Globals.extendoDistance - 200;
                break;
            case LIMELIGHT_TAKE_POSE:
                targetPosition = Math.min(Globals.extendoDistance + 200, MAXIMUM);
                break;
            case LIMELIGHT_TAKE_POSE_AFTER_THROWING:
                targetPosition = Math.min(currentPosition + 200, MAXIMUM);
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


    public boolean canOuttakeSample(){
        return currentPosition <= 200;
    }

    public boolean canPutIntakeDown(){
        return currentPosition >= 200;
    }

    public boolean limelightPoseFinished(){
        if(targetPosition == MAXIMUM){
            return currentPosition >= MAXIMUM - 20;
        }
        return currentPosition >= (Globals.extendoDistance - 20);
    }

    public boolean limelightRetractPoseFinished(){
        return  currentPosition <= (Globals.extendoDistance - 200 + 20);
    }

    public boolean limelightTakePoseFinished(){
        if(targetPosition == MAXIMUM){
            return  currentPosition >= MAXIMUM - 20;
        }
        return currentPosition >= (Globals.extendoDistance + 200 - 20);
    }

    public boolean limelightTakePoseAfterThrowingFinished(){
        if(targetPosition == MAXIMUM){
            return  currentPosition >= MAXIMUM - 20;
        }
        return currentPosition >= targetPosition - 20;
    }

    public boolean retractFinished(){
        return currentPosition <= 20;
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

        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        double power = extendo_pid.calculate(currentPosition, targetMotionProfile);
        extendoMotor.setPower(power);
    }



    public void testPID(){
        currentPosition = extendoMotor.getCurrentPosition();
        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        double power = extendo_pid.calculate(currentPosition, targetPosition);
        extendoMotor.setPower(power);
    }




}
