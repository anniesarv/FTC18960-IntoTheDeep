package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivetrainConstants {
    public static double LATERAL_KP = 0.04;
    public static double LATERAL_KI = 0.00;
    public static double LATERAL_KD = 0.00;
    public static double LATERAL_KF = 0.07;

    public static double ROTATION_KP = 0.3;
    public static double ROTATION_KI = 0.00;
    public static double ROTATION_KD = 0.00;
    public static double ROTATION_KF = 0.05;

    public static double LATERAL_POSITION_TOLERANCE = 0.5;
    public static double LATERAL_VELOCITY_TOLERANCE = 10.0;

    public static double ROTATION_POSITION_TOLERANCE = 0.05;
    public static double ROTATION_VELOCITY_TOLERANCE = 2.0;


}
