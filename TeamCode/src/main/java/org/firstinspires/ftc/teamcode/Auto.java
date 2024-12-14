package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.DropSample;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.commands.HorizontalExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.HorizontalWristCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.commands.PlaceSample;
import org.firstinspires.ftc.teamcode.commands.VerticalExtensionCancel;
import org.firstinspires.ftc.teamcode.commands.VerticalExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalWristCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist;

@Autonomous
public class Auto extends OpMode {
    private Drivetrain drive;
    private HorizontalExtension horizontalExtension;
    private VerticalExtension verticalExtension;
    private Intake intake;
    private HorizontalWrist horizontalWrist;
    private VerticalWrist verticalWrist;
    private Claw claw;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        drive = new Drivetrain(hardwareMap);
        horizontalExtension = new HorizontalExtension(hardwareMap);
        verticalExtension = new VerticalExtension(hardwareMap);
        verticalWrist = new VerticalWrist(hardwareMap);
        horizontalWrist = new HorizontalWrist(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

        drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)));


    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new SequentialCommandGroup(),
                new ParallelCommandGroup(
                        new PIDToPosition(drive, new Pose2d(-4.0, -20.0, Rotation2d.fromDegrees(135.0))),
                        new PlaceSample(verticalExtension, verticalWrist, claw, intake).withTimeout(2000)
                ),
                new DropSample(verticalExtension, verticalWrist, claw).withTimeout(1500),
                new InstantCommand(() -> intake.stop()),
                new PIDToPosition(drive, new Pose2d(-50.0, 4.0, Rotation2d.fromDegrees(0.0)))
        ));
    }

    @Override
    public void loop() {


        Pose2d pose = drive.getPose();

        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("degrees", pose.getRotation().getDegrees());

        telemetry.addData("elevator position", verticalExtension.getExtensionPosition());
        telemetry.addData("extension position", horizontalExtension.getExtensionPosition());
        telemetry.addData("extension speed", horizontalExtension.getExtensionSpeed());
        telemetry.update();

        CommandScheduler.getInstance().run();

    }
}
