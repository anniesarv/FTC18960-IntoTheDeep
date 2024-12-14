package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private Drivetrain drive;
    private HorizontalExtension horizontalExtension;
    private Intake intake;
    private GamepadEx driver;
    private GamepadEx driver2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        drive = new Drivetrain(hardwareMap);
        horizontalExtension = new HorizontalExtension(hardwareMap);
        intake = new Intake(hardwareMap);
        driver = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        drive.fieldCentricDrive(
                new ChassisSpeeds(driver.getLeftX(), driver.getLeftY(), -driver.getRightX())
        );

        horizontalExtension.setExtensionSpeed(driver2.getRightY());

        intake.intake(1.0);

        //horizontalExtension.intake();

        Pose2d pose = drive.getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("degrees", pose.getRotation().getDegrees());

        telemetry.addData("extension position", horizontalExtension.getExtensionPosition());
        telemetry.addData("extension speed", horizontalExtension.getExtensionSpeed());
        telemetry.update();

        CommandScheduler.getInstance().run();

    }
}
