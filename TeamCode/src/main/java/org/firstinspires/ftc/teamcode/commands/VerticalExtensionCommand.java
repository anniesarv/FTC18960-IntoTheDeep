package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

import java.util.function.DoubleSupplier;

public class VerticalExtensionCommand extends CommandBase {
    private VerticalExtension verticalExtension;
    private DoubleSupplier speed;

    public VerticalExtensionCommand(VerticalExtension verticalExtension, DoubleSupplier speed) {
        this.verticalExtension = verticalExtension;
        this.speed = speed;
        addRequirements(verticalExtension);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        verticalExtension.setExtensionSpeed(speed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        verticalExtension.setExtensionSpeed(0.0);
    }
}
