package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
    private Intake intake;
    private DoubleSupplier speed;

    public IntakeCommand(Intake intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intake(speed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
