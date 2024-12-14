package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;

import java.util.function.DoubleSupplier;

public class FieldCentricDrive extends CommandBase {
    private Drivetrain drive;
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private DoubleSupplier headingSpeed;

    public FieldCentricDrive(Drivetrain drive, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier headingSpeed) {
        this.drive = drive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.headingSpeed = headingSpeed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drive.fieldCentricDrive(
                new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), headingSpeed.getAsDouble())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
