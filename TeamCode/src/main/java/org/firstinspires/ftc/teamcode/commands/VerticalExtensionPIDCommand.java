package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.VerticalConstants;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

import java.util.function.DoubleSupplier;

public class VerticalExtensionPIDCommand extends CommandBase {
    private VerticalExtension verticalExtension;
    private PIDFController controller;
    private double setpoint;

    public VerticalExtensionPIDCommand(VerticalExtension verticalExtension, double setpoint) {
        this.verticalExtension = verticalExtension;
        this.setpoint = setpoint;
        controller = new PIDFController(VerticalConstants.KP, VerticalConstants.KI, VerticalConstants.KD, 0.0);
        controller.setTolerance(VerticalConstants.POSITION_TOLERANCE, VerticalConstants.VELOCITY_TOLERANCE);
        addRequirements(verticalExtension);
    }

    @Override
    public void initialize() {
        controller.calculate(verticalExtension.getExtensionPosition(), setpoint);
    }

    @Override
    public void execute() {
        verticalExtension.setExtensionSpeed(
                controller.calculate(verticalExtension.getExtensionPosition(), setpoint)
        );

    }

    @Override
    public boolean isFinished() {
        //return controller.atSetPoint();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        verticalExtension.setExtensionSpeed(0.0);
    }
}
