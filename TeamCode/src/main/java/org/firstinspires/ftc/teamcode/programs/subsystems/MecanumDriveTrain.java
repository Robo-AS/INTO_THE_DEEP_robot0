package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.WSubsystem;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector2D;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class MecanumDriveTrain extends WSubsystem implements Drivetrain {
    //private final Robot robot = Robot.getInstance();
    private static MecanumDriveTrain instance = null;
    public CachingDcMotorEx dtFrontLeftMotor, dtFrontRightMotor, dtBackLeftMotor, dtBackRightMotor;
    public double sensor;

    double[] ws = new double[4];
    private int frontLeft = 3, frontRight = 1, backLeft = 2, backRight = 0;


    public double ks = 0;


    public static MecanumDriveTrain getInstance(){
        if (instance == null) {
            instance = new MecanumDriveTrain();
        }
        return instance;
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        dtBackLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backLeftMotor"));
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtFrontLeftMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontLeftMotor"));
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtBackRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRightMotor"));
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtFrontRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontRightMotor"));
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        sensor = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }


    @Override
    public void set(Pose pose) {
        set(pose, 0);
    }

    public void set(double forwardSpeed, double strafeSpeed, double turnSpeed, double gyroAngle) {
        Vector2D input = new Vector2D(forwardSpeed, strafeSpeed).rotate(-gyroAngle);

        double actualks = ks; // *12/getVoltage();

        forwardSpeed = Range.clip(input.x, -1, 1);
        strafeSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[frontLeft] = (forwardSpeed - strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum((forwardSpeed - strafeSpeed + turnSpeed));
        wheelSpeeds[frontRight] = (forwardSpeed + strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed + turnSpeed);
        wheelSpeeds[backLeft] = (forwardSpeed + strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed - turnSpeed);
        wheelSpeeds[backRight] = (forwardSpeed - strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed - strafeSpeed - turnSpeed);  //(forwardSpeed - strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum((forwardSpeed - strafeSpeed + turnSpeed));

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));


        if (max > 1) {
            wheelSpeeds[frontLeft] /= max;
            wheelSpeeds[frontRight] /= max;
            wheelSpeeds[backLeft] /= max;
            wheelSpeeds[backRight] /= max;
        }

        ws[frontLeft] = wheelSpeeds[frontLeft];
        ws[frontRight] = wheelSpeeds[frontRight];
        ws[backLeft] = wheelSpeeds[backLeft];
        ws[backRight] = wheelSpeeds[backRight];

        dtFrontLeftMotor.setPower(ws[frontLeft]);
        dtFrontRightMotor.setPower(ws[frontRight]);
        dtBackLeftMotor.setPower(ws[backLeft]);
        dtBackRightMotor.setPower(ws[backRight]);
    }


    public void set(Pose pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
    }


    @Override
    public void periodic() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }


    public double getVoltage(){
        return sensor;
    }


}
