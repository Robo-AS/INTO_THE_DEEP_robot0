package org.firstinspires.ftc.teamcode.programs.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Lift extends SubsystemBase{
    private static Lift instance = null;
    public CachingDcMotorEx liftMotor, followerMotor;
    private ElapsedTime time = new ElapsedTime();


    public enum LiftState{
        LOW_BASKET,
        HIGH_BASKET,
        LOW_RUNG,
        HIGH_RUNG,
        ARM_MOVEMENT,
        IDLE
    }

    public LiftState liftState = LiftState.IDLE;
    public int LOW_BASKET = 0;
    public int HIGH_BASKET = 1200;
    public int LOW_RUNG= 0;
    public int HIGH_RUNG = 600;
    public int ARM_MOVEMENT = 0;
    public int IDLE = 0;



    private PIDController lift_pid;
//    public static double p_lift = 0.0056, d_lift = 0.0056, i_lift = 0.0001; //the d is too bog as last tested
//    public static double f_lift = 0.04; //basically a constant you tune until the lift holds itself in place(doesn't fall due to gravity)

    public static int targetPosition;
    public static int currentPosition;

    public static double p_lift = 0.0056, d_lift = 0.00009, i_lift = 0.1;

    MotionProfile profile;
    public static int previousTarget = 0;
    public static double maxVelocityUP = 10000000, maxAccelerationUP = 200000;
    public static double maxVelocityDOWN = 1000000, maxAccelerationDOWN = 4000;


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
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        followerMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "followerMotor"));
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        followerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void initialize(){
        liftState = LiftState.IDLE;
        lift_pid.reset();
    }


    public void loop(){
        currentPosition = liftMotor.getCurrentPosition();

        if(targetPosition != previousTarget) {
            if(targetPosition > previousTarget) { //profile pentru extindere
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(previousTarget, 0),
                        new MotionState(targetPosition, 0),
                        maxVelocityUP,
                        maxAccelerationUP
                );
            }

            if(targetPosition < previousTarget){ //profile pentru retragere
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        new MotionState(previousTarget, 0),
                        new MotionState(targetPosition, 0),
                        maxVelocityDOWN,
                        maxAccelerationDOWN
                );
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

            case ARM_MOVEMENT:
                targetPosition = ARM_MOVEMENT;
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
}
