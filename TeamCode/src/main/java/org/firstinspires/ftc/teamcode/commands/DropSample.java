package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist;

public class DropSample extends SequentialCommandGroup {
    public DropSample(VerticalExtension verticalExtension, VerticalWrist verticalWrist, Claw claw) {

        addCommands(
                new SequentialCommandGroup(
                        new ClawCommand(claw, 1.0),
                        new WaitCommand(250),
                        new ParallelCommandGroup(
                                new VerticalWristCommand(verticalWrist, 0.84),
                                //new VerticalExtensionPIDCommand(verticalExtension, 0.0)
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new VerticalExtensionPIDCommand(verticalExtension, 0.0)
                                )

                        )
                )
        );

        addRequirements(verticalExtension, verticalWrist, claw);
    }
}
