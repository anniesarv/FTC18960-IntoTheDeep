package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist;

public class VerticalWristCommand extends CommandBase {
    private VerticalWrist verticalWrist;
    private double speed;

    public VerticalWristCommand(VerticalWrist verticalWrist, double speed) {
        this.verticalWrist = verticalWrist;
        this.speed = speed;
        addRequirements(verticalWrist);
    }

    @Override
    public void initialize() {
        verticalWrist.setPosition(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
