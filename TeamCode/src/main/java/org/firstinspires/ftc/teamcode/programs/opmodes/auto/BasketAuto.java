//package org.firstinspires.ftc.teamcode.programs.opmodes.auto;
//
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.programs.commandbase.ArmCommands.SetClawStateCommand;
//import org.firstinspires.ftc.teamcode.programs.commandbase.LiftCommands.SetLiftStateCommand;
//import org.firstinspires.ftc.teamcode.programs.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.programs.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.programs.util.Robot;
//
//import java.util.ArrayList;
//
//@Autonomous(name = "BasketAuto")
//public class BasketAuto extends CommandOpMode {
//    private final Robot robot = Robot.getInstance();
//
//    private final ArrayList<PathChain> paths = new ArrayList<>();
//
//
//    public void generatePath() {
//        robot.follower.setStartingPose(new Pose(7, 112, Math.toRadians(-90)));
//
//        paths.add(
//                robot.follower.pathBuilder()
//                        .addPath(
//                                new BezierLine(
//                                        new Point(7, 112,  Point.CARTESIAN),
//                                        new Point(14, 127, Point.CARTESIAN)
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
//                        .build());
//    }
//
//
//    @Override
//    public void initialize() {
//        super.reset();
//        robot.initializeHardware(hardwareMap);
//        robot.initializeRobot();
////        robot.follower.setMaxPower(1);
//
//
//
//        generatePath();
//
//
//        schedule(
//                new RunCommand(()-> robot.follower.update()),
//
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new FollowPath(robot.follower, paths.get(0)),
//                                new SetLiftStateCommand(Lift.LiftState.HIGH_BASKET)
//                        ),
//                        new SetClawStateCommand(Arm.ClawState.OPEN)
//
//                )
//        );
//    }
//
//    @Override
//    public void run(){
//        super.run();
//
//        robot.loop();
//        robot.lift.loop();
//    }
//
//
//}
