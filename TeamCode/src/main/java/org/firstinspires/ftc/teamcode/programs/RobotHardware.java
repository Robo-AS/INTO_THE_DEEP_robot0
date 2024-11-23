package org.firstinspires.ftc.teamcode.programs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;


public class RobotHardware {

    private static RobotHardware instance = null;
    private boolean instanceEnabled;

    private HardwareMap hardwareMap;

    //Intake:
    public CachingDcMotorEx extendoMotor;

    //Brush
    public CachingDcMotorEx brushMotor;
    public CachingServo brushAngleServo;
    public CachingServo brushSampleServo;

    public ColorSensor colorSensor0;




    public static RobotHardware getInstance(){
        if(instance == null){
            instance = new RobotHardware();
        }
        instance.instanceEnabled = true;
        return instance;
    }

    public void initialize(final HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        //Intake
        extendoMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extendoMotor"));
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        brushMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "brushMotor"));
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brushMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        brushSampleServo = new CachingServo(hardwareMap.get(Servo.class, "brushSampleServo"));
        brushSampleServo.setDirection(Servo.Direction.FORWARD);


        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        telemetry.addData("Sensor value", colorSensor0.red());

    }
}
