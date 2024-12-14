package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Intake extends SubsystemBase {
    private CRServoImplEx intakeRight, intakeLeft;

    public Intake(HardwareMap hardwareMap) {
        intakeLeft = hardwareMap.get(CRServoImplEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServoImplEx.class, "intakeRight");

        intakeLeft.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));
        intakeRight.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));
    }

    @Override
    public void periodic() {

    }

    public void intake(double speed) {
        intakeLeft.setPower(-speed);
        intakeRight.setPower(speed);
    }

    public void stop() {
        intakeLeft.setPower(0.0);
        intakeRight.setPower(0.0);
    }
}
