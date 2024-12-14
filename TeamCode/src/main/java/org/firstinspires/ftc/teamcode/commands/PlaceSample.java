package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist;

public class PlaceSample extends SequentialCommandGroup {
    public PlaceSample(VerticalExtension verticalExtension, VerticalWrist verticalWrist, Claw claw, Intake intake) {

        addCommands(
                new SequentialCommandGroup(
                        new ClawCommand(claw, 0.0),
                        new WaitCommand(250),
                        new InstantCommand(() -> intake.intake(1.0)),
                        new VerticalWristCommand(verticalWrist, 0.0),
                        //new ParallelCommandGroup(
                                new VerticalExtensionPIDCommand(verticalExtension, 28.0)
                                /*
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                )

                                 */

                        //)
                )
        );

        addRequirements(verticalExtension, verticalWrist, claw);
    }
}
