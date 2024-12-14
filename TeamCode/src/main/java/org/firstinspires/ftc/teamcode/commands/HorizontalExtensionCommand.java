package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;

import java.util.function.DoubleSupplier;

public class HorizontalExtensionCommand extends CommandBase {
    private HorizontalExtension horizontalExtension;
    private DoubleSupplier speed;

    public HorizontalExtensionCommand(HorizontalExtension horizontalExtension, DoubleSupplier speed) {
        this.horizontalExtension = horizontalExtension;
        this.speed = speed;
        addRequirements(horizontalExtension);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        horizontalExtension.setExtensionSpeed(speed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        horizontalExtension.setExtensionSpeed(0.0);
    }
}
