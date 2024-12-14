package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HorizontalExtension extends SubsystemBase {
    private Motor extension;
    // private DigitalChannel colour0, colour1;
    // private DigitalChannel extensionLimit;

    private double extensionPosition = 0.0;

    public HorizontalExtension(HardwareMap hardwareMap) {
        extension = new Motor(hardwareMap, "horizontalExtension", Motor.GoBILDA.RPM_1150);

        extension.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extension.stopAndResetEncoder();
    }

    @Override
    public void periodic() {
        extensionPosition = extension.getCurrentPosition() * 0.03272804054;

    }

    public void setExtensionSpeed(double speed) {
        /*
        double currentPosition = getExtensionPosition();

        if ((currentPosition < HorizontalConstants.EXTENSION_LOWER_LIMIT && speed <= 0.0) || (currentPosition > HorizontalConstants.EXTENSION_UPPER_LIMIT && speed > 0.0)) {
            setRawExtensionSpeed(0.0);
        } else {
            setRawExtensionSpeed(
                    speed + HorizontalConstants.KG
            );
        }
         */

        setRawExtensionSpeed(speed);

    }

    public double getExtensionPosition() {
        return extensionPosition;
    }

    public void setRawExtensionSpeed(double speed) {
        extension.set(-speed);
    }

    public double getExtensionSpeed() {
        return extension.get();
    }



}
