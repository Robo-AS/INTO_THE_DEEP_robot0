package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Brush extends SubsystemBase{
    private HardwareMap hardwareMap;

    private CachingDcMotorEx brushMotor;
    private CachingServo brushUpDownServo;
    private CachingServo brushSampleServo;
    private ColorSensor colorSensor0;

    public enum BrushState {
        INTAKING,
        KEEPING,
        THROWING,
        IDLE;
    }

    public enum BrushAngle{
        UP,
        DOWN;
    }





    public void initializeHardware(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        brushMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "brushMotor"));
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brushMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        brushUpDownServo = new CachingServo(hardwareMap.get(Servo.class, "brushAngleServo"));
        brushUpDownServo.setDirection(Servo.Direction.FORWARD);

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));
        brushSampleServo.setDirection(Servo.Direction.FORWARD);

        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");

    }


    public void initialize() {

    }



}
