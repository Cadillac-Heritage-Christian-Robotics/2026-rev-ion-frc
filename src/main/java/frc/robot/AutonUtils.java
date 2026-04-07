package frc.robot;

import java.util.Map;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public final class AutonUtils {

    public static final Map<String, Pose2d> AUTON_TARGET_POSES = Map.ofEntries(
        // Blue North
        Map.entry("BlueNorthAlpha",   new Pose2d(2.525, 4.456, Rotation2d.fromDegrees(-45))),
        Map.entry("BlueNorthBravo",   new Pose2d(2.525, 4.456, Rotation2d.fromDegrees(-45))),
        Map.entry("BlueNorthDefault", new Pose2d(2.525, 4.456, Rotation2d.fromDegrees(-45))),
        // Blue South
        Map.entry("BlueSouthAlpha",   new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(22))),
        Map.entry("BlueSouthBravo",   new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(22))),
        Map.entry("BlueSouthDefault", new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(22)))
    );

    public static Pose2d getDesiredPose(String strategy) throws IllegalArgumentException {
        Pose2d targetPose = AUTON_TARGET_POSES.get(strategy);

        if (targetPose == null) {
            throw new IllegalArgumentException("No starting pose found for strategy: " + strategy);
        }

        return targetPose;
    }
}