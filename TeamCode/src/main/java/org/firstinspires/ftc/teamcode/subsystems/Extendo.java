package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Extendo extends SubsystemBase {
    private HardwareMap hardwareMap;
    private CachingDcMotorEx extendoMotor;

    private PIDController extendoPIDController;
    public static double p_extendo = 0, i_extendo = 0, d_extendo = 0;

    public enum extendoState{
        EXTENDING,
        RETRACTING;
    }

    public void initializeHarware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        extendoMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendoMotor"));
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initialize() {

    }
}
