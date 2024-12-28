package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Config
public class Extendo extends SubsystemBase {
    private static Extendo instance = null;
    public CachingDcMotorEx extendoMotor;

    public enum ExtendoState{
        EXTENDING_MINIMUM,
        RETRACTING
    }

    public ExtendoState extendoState = ExtendoState.IDLE;
    public int EXTENDING_MINIMUM = 350;
    public int RETRACTING = 0;


    private PIDController extendo_pid;
    public static double p_extendo = 0.089, i_extendo = 0, d_extendo = 0;

    public static double targetPosition;
    public static double currentPosition;

    public static double joystickConstant = 1.0;
    public static double minPosition = 350, maxPosition = 1300;



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
    }


    public void loop(double joystickYCoord){
        currentPosition = extendoMotor.getCurrentPosition();

        if(extendoState == ExtendoState.EXTENDING_MINIMUM)
            updateTargetPosition(joystickYCoord);

        extendo_pid.setPID(p_extendo, i_extendo, d_extendo);
        double power = extendo_pid.calculate(currentPosition, targetPosition);
        extendoMotor.setPower(power);

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
        targetPosition += joystickYCoord * joystickConstant;
        targetPosition = Math.max(minPosition, Math.min(maxPosition, targetPosition)); //limita de prosti
    }

    public double getTargetPosition(){
        return targetPosition;
    }
}
