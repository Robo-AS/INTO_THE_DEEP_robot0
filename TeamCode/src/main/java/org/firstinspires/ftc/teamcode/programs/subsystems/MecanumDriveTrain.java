package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.programs.util.Robot;
import org.firstinspires.ftc.teamcode.wrappers.Drivetrain;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.wrappers.geometry.Pose;
import org.firstinspires.ftc.teamcode.wrappers.geometry.Vector2D;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class MecanumDriveTrain extends WSubsystem implements Drivetrain {
    //private final Robot robot = Robot.getInstance();
    private static MecanumDriveTrain instance = null;
    public CachingDcMotorEx dtFrontLeftMotor, dtFrontRightMotor, dtBackLeftMotor, dtBackRightMotor;
    public double sensor;

    double[] ws = new double[4];
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
        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dtBackRightMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRightMotor"));
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        double actualks = ks *12/getVoltage();

        forwardSpeed = Range.clip(input.x, -1, 1);
        strafeSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = (forwardSpeed - strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed - strafeSpeed - turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = (forwardSpeed + strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed + turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = (forwardSpeed + strafeSpeed - turnSpeed)*(1-actualks) + actualks*Math.signum(forwardSpeed + strafeSpeed - turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = (forwardSpeed - strafeSpeed + turnSpeed)*(1-actualks) + actualks*Math.signum((forwardSpeed - strafeSpeed + turnSpeed));

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));


        if (max > 1) {
            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] /= max;
        }

        ws[0] = wheelSpeeds[0];
        ws[1] = wheelSpeeds[1];
        ws[2] = wheelSpeeds[2];
        ws[3] = wheelSpeeds[3];

        dtFrontLeftMotor.setPower(ws[0]);
        dtFrontRightMotor.setPower(ws[1]);
        dtBackLeftMotor.setPower(ws[2]);
        dtBackRightMotor.setPower(ws[3]);
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
