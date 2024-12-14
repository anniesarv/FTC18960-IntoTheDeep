package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;

public class HorizontalWristCommand extends CommandBase {
    private HorizontalWrist horizontalWrist;
    private double speed;

    public HorizontalWristCommand(HorizontalWrist horizontalWrist, double speed) {
        this.horizontalWrist = horizontalWrist;
        this.speed = speed;
        addRequirements(horizontalWrist);
    }

    @Override
    public void initialize() {
        horizontalWrist.setPosition(speed);
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
