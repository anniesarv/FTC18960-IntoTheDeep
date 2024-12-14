package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.VerticalConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

public class PIDToPosition extends CommandBase {
    private Drivetrain drive;
    private PIDFController xController;
    private PIDFController yController;
    private PIDController headingController;
    private Pose2d setpoint;

    public PIDToPosition(Drivetrain drive, Pose2d setpoint) {
        this.drive = drive;
        this.setpoint = setpoint;

        xController = new PIDFController(DrivetrainConstants.LATERAL_KP, DrivetrainConstants.LATERAL_KI, DrivetrainConstants.LATERAL_KD, 0.0);
        yController = new PIDFController(DrivetrainConstants.LATERAL_KP, DrivetrainConstants.LATERAL_KI, DrivetrainConstants.LATERAL_KD, 0.0);
        headingController = new PIDController(DrivetrainConstants.ROTATION_KP, DrivetrainConstants.ROTATION_KI, DrivetrainConstants.ROTATION_KD);

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(DrivetrainConstants.LATERAL_POSITION_TOLERANCE, DrivetrainConstants.LATERAL_VELOCITY_TOLERANCE);
        yController.setTolerance(DrivetrainConstants.LATERAL_POSITION_TOLERANCE, DrivetrainConstants.LATERAL_VELOCITY_TOLERANCE);
        headingController.setTolerance(DrivetrainConstants.LATERAL_POSITION_TOLERANCE, DrivetrainConstants.LATERAL_VELOCITY_TOLERANCE);


        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        xController.calculate(currentPose.getX(), setpoint.getX());
        yController.calculate(currentPose.getY(), setpoint.getY());
        headingController.calculate(currentPose.getHeading(), setpoint.getHeading());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        double xFeedback = xController.calculate(currentPose.getX());
        xFeedback += Math.signum(xFeedback) * DrivetrainConstants.LATERAL_KF;
        double yFeedback = -yController.calculate(currentPose.getY());
        yFeedback += Math.signum(yFeedback) * DrivetrainConstants.LATERAL_KF;
        double headingFeedback = headingController.calculate(currentPose.getRotation().getRadians());
        headingFeedback += Math.signum(headingFeedback) * DrivetrainConstants.LATERAL_KF;

        drive.fieldCentricDrive(new ChassisSpeeds(
                yFeedback, xFeedback, headingFeedback
        ));

    }

    @Override
    public boolean isFinished() {
        return xController.atSetPoint() && yController.atSetPoint() && headingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
