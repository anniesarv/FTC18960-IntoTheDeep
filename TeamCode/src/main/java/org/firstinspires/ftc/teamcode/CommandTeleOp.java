package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.DropSample;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.commands.HorizontalExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.HorizontalWristCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PlaceSample;
import org.firstinspires.ftc.teamcode.commands.VerticalExtensionCancel;
import org.firstinspires.ftc.teamcode.commands.VerticalExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalExtensionPIDCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalWristCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class CommandTeleOp extends OpMode {
    private Drivetrain drive;
    private HorizontalExtension horizontalExtension;
    private VerticalExtension verticalExtension;
    private Intake intake;
    private HorizontalWrist horizontalWrist;
    private VerticalWrist verticalWrist;
    private Claw claw;
    private GamepadEx driver;
    private GamepadEx driver2;

    private boolean elevatorInterrupt = false;

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
        driver = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        drive.setDefaultCommand(
                new FieldCentricDrive(
                        drive,
                        () -> driver.getLeftX(),
                        () -> driver.getLeftY(),
                        () -> -driver.getRightX()
                )
        );

        horizontalExtension.setDefaultCommand(
                new HorizontalExtensionCommand(horizontalExtension, () -> driver2.getRightY())
        );

        verticalExtension.setDefaultCommand(
                new VerticalExtensionCommand(verticalExtension, () -> driver2.getLeftY())
        );

        intake.setDefaultCommand(
                new IntakeCommand(intake, () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
        );
    }

    @Override
    public void loop() {
        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new HorizontalWristCommand(horizontalWrist, 0.0));
        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new HorizontalWristCommand(horizontalWrist, 1.0));

        //driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new VerticalExtensionPIDCommand(verticalExtension, 25.0));
        //driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new VerticalExtensionPIDCommand(verticalExtension, 0.0));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new VerticalWristCommand(verticalWrist, 0.0));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new VerticalWristCommand(verticalWrist, 1.0));

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ClawCommand(claw, 0.0));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ClawCommand(claw, 1.0));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new PlaceSample(verticalExtension, verticalWrist, claw, intake));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new DropSample(verticalExtension, verticalWrist, claw));

        if (!elevatorInterrupt && Math.abs(driver2.getLeftY()) > 0.01) {
            elevatorInterrupt = true;
            CommandScheduler.getInstance().schedule(new VerticalExtensionCancel(verticalExtension));
        } else if (Math.abs(driver2.getLeftY()) <= 0.01) {
            elevatorInterrupt = false;
        }

        Pose2d pose = drive.getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("degrees", pose.getRotation().getDegrees());

        telemetry.addData("x speed", driver.getLeftX());
        telemetry.addData("y speed", driver.getLeftY());
        telemetry.addData("heading speed", driver.getRightX());

        telemetry.addData("elevator position", verticalExtension.getExtensionPosition());
        telemetry.addData("extension position", horizontalExtension.getExtensionPosition());
        telemetry.addData("extension speed", horizontalExtension.getExtensionSpeed());
        telemetry.update();

        CommandScheduler.getInstance().run();

    }
}
