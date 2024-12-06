package org.firstinspires.ftc.teamcode.IntakeTests;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimplestCommand extends InstantCommand {
    public SimplestCommand(Telemetry telemetry) {
        super(
                () -> {
                    telemetry.addData("Message", "idk");
                    telemetry.update();
                }
        );
    }
}