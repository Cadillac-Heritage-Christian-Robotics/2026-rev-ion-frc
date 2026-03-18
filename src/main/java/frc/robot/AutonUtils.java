package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TargetPositions;


public final class AutonUtils {
    public static final Map<String, Pose2d> AUTON_START_POSES = Map.of(
        "BlueNorthAlpha", new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(83.478)),
        "BlueNorthBravo", new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(119.775)),
        "BlueSouthAlpha", new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(-104.069)),
        "BlueSouthBravo", new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(119.177)),
        "RedNorthAlpha", new Pose2d(13.602, 4.808, Rotation2d.fromDegrees(110.184)),
        "RedNorthBravo", new Pose2d(13.602, 4.808, Rotation2d.fromDegrees(80.221)),
        "RedSouthAlpha", new Pose2d(13.780, 3.250, Rotation2d.fromDegrees(-42.038)),
        "RedSouthBravo", new Pose2d(13.780, 3.250, Rotation2d.fromDegrees(-37.443))
    );

    public static Pose2d getShootPosition() {
        var alliance = DriverStation.getAlliance();
        // TODO - if not set then PANIC
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return TargetPositions.RED_SHOOT;
        }

        return TargetPositions.BLUE_SHOOT;
    }

    public static Pose2d getStartingPose(String strategy) {
       return AUTON_START_POSES.getOrDefault(strategy, new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(29.866)));
    }
}