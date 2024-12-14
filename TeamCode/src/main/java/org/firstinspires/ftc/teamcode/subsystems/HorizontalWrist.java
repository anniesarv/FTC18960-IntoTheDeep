package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HorizontalWrist extends SubsystemBase {
    private ServoImplEx wrist;

    public HorizontalWrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(ServoImplEx.class, "horizontalWrist");

        wrist.setPwmRange(new PwmControl.PwmRange(510.0, 2490.0));
    }

    @Override
    public void periodic() {

    }

    public void setPosition(double position) {
        wrist.setPosition(position);
    }



}
