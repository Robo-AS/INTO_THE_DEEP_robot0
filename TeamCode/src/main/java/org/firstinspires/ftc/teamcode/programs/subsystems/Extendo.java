package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Config
public class Extendo extends SubsystemBase {
    private static Extendo instance = null;
    public CachingDcMotorEx extendoMotor;
    private ElapsedTime time = new ElapsedTime();

    public enum ExtendoState{
        EXTENDING_MINIMUM,
        RETRACTING
    }
    private TrapezoidProfile profile;

    public ExtendoState extendoState = ExtendoState.RETRACTING;
    public int EXTENDING_MINIMUM = 350;
    public int RETRACTING = 0;


    private final PIDController extendo_pid;
    public static double p_extendo = 0.007, i_extendo = 0.11, d_extendo = 0.00006;

    public static int targetPosition = 0;
    public static int currentPosition = 0;

    public static double joystickConstant = 50;
    public static double exponentialJoystickCoef;
    public static int minPosition = 550, maxPosition = 1700;
    public static double contantTerm = 0.6, liniarCoefTerm = 0.7;

    public static double maxVelocity = 1, maxAcceleration = 1;
    public static int profileDistance;
    private static int targetPrevious = 0;
    private boolean trajectoryIsNotDone = false;

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
        extendoState = ExtendoState.RETRACTING;
        extendo_pid.reset();
        time.reset();
        targetPosition = 0;
        targetPrevious = 0;
        currentPosition = 0;
        profileDistance = 0;
        trajectoryIsNotDone = false;
    }


    public void loop(double joystickYCoord){
        currentPosition = extendoMotor.getCurrentPosition();
        if(targetPosition != targetPrevious){
            trajectoryIsNotDone = true;
            targetPrevious = targetPosition;
            profileDistance = targetPosition - currentPosition;
            time.reset();
        }

        if(trajectoryIsNotDone){

            targetPosition = (int) motionProfile(maxAcceleration, maxVelocity, profileDistance, time.seconds());

            if(Math.abs(targetPosition - currentPosition) < 5){
                trajectoryIsNotDone = false;
            }
        }

        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        double power = extendo_pid.calculate(currentPosition, targetPosition);
        extendoMotor.setPower(power);



        if(extendoState == ExtendoState.EXTENDING_MINIMUM)
            updateTargetPosition(joystickYCoord);

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
        }

    }

    public void updateTargetPosition(double joystickYCoord){
        exponentialJoystickCoef = (Math.pow(joystickYCoord, 3) + liniarCoefTerm * joystickYCoord) * contantTerm;
        targetPosition += (int)(exponentialJoystickCoef * joystickConstant);
        targetPosition = Math.max(minPosition, Math.min(maxPosition, targetPosition)); //limita de prosti
    }

    public int getTargetPosition(){
        return targetPosition;
    }
    public double getExponentialJoystickCoef() {return exponentialJoystickCoef; }

    public static boolean canOuttakeSample(){
        return currentPosition <= 30;
    }


    private double motionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        double accelerationTime = maxVelocity / maxAcceleration;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        if (accelerationDistance > distance / 2) {
            accelerationTime = Math.sqrt((distance / 2) / (0.5 * maxAcceleration));
            maxVelocity = maxAcceleration * accelerationTime;
        }

        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;
        double totalTime = 2 * accelerationTime + cruiseTime;

        if (elapsedTime >= totalTime) {
            return distance;
        }
        else if (elapsedTime < accelerationTime) {
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }
        else if (elapsedTime < (accelerationTime + cruiseTime)) {
            double cruiseElapsedTime = elapsedTime - accelerationTime;
            return accelerationDistance + maxVelocity * cruiseElapsedTime;
        }
        else {
            double decelerationElapsedTime = elapsedTime - (accelerationTime + cruiseTime);
            return accelerationDistance + cruiseDistance +
                    maxVelocity * decelerationElapsedTime -
                    0.5 * maxAcceleration * Math.pow(decelerationElapsedTime, 2);
        }
    }
}
