package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw extends SubsystemBase {
    private ServoImplEx wrist;

    public Claw(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(ServoImplEx.class, "claw");

        //wrist.setPwmRange(new PwmControl.PwmRange(510.0, 2490.0));
    }

    @Override
    public void periodic() {

    }

    public void setPosition(double position) {
        wrist.setPosition(position);
    }



}
