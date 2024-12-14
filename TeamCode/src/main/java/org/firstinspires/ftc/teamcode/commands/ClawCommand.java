package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist;

public class ClawCommand extends CommandBase {
    private Claw claw;
    private double speed;

    public ClawCommand(Claw claw, double speed) {
        this.claw = claw;
        this.speed = speed;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setPosition(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
