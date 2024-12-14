package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.VerticalConstants;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

public class VerticalExtensionCancel extends CommandBase {
    private VerticalExtension verticalExtension;

    public VerticalExtensionCancel(VerticalExtension verticalExtension) {
        this.verticalExtension = verticalExtension;
        addRequirements(verticalExtension);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        //return controller.atSetPoint();
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        verticalExtension.setExtensionSpeed(0.0);
    }
}
