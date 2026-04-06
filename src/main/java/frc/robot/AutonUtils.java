package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetPositions;


public final class AutonUtils {

    public static final Map<String, Pose2d> AUTON_TARGET_POSES = Map.ofEntries(
        // Blue North
        Map.entry("BlueNorthAlpha",   new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(-45))),
        Map.entry("BlueNorthBravo",   new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(-119.775))),
        Map.entry("BlueNorthDefault", new Pose2d(2.525, 4.456, Rotation2d.fromDegrees(-18.759))),
        // Blue South
        Map.entry("BlueSouthAlpha",   new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(-104.069))),
        Map.entry("BlueSouthBravo",   new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(119.177))),
        Map.entry("BlueSouthDefault", new Pose2d(2.938, 3.348, Rotation2d.fromDegrees(-104.069))),
        // Red North
        Map.entry("RedNorthAlpha",    new Pose2d(13.602, 4.808, Rotation2d.fromDegrees(110.184))),
        Map.entry("RedNorthBravo",    new Pose2d(13.602, 4.808, Rotation2d.fromDegrees(80.221))),
        Map.entry("RedNorthDefault",  new Pose2d(13.602, 4.808, Rotation2d.fromDegrees(110.184))),
        // Red South
        Map.entry("RedSouthAlpha",    new Pose2d(13.780, 3.250, Rotation2d.fromDegrees(-42.038))),
        Map.entry("RedSouthBravo",    new Pose2d(13.780, 3.250, Rotation2d.fromDegrees(-37.443))),
        Map.entry("RedSouthDefault",  new Pose2d(13.532, 3.446, Rotation2d.fromDegrees(163.718)))
    );
    public static Pose2d getShootPosition() {
        var alliance = DriverStation.getAlliance();
        // TODO - if not set then PANIC
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return TargetPositions.RED_SHOOT;
        }

        return TargetPositions.BLUE_SHOOT;
    }

    public static Pose2d getDesiredPose(String strategy) throws IllegalArgumentException {
        Pose2d pose = AUTON_TARGET_POSES.get(strategy);
        if (pose == null) {
            throw new IllegalArgumentException("No starting pose found for strategy: " + strategy);
        }
        return pose;
    }
}