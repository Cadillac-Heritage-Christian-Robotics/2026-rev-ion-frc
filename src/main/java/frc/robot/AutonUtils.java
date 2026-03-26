package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetPositions;


public final class AutonUtils {
    public static final Map<String, Pose2d> AUTON_START_POSES = Map.ofEntries(
        // Blue North
        Map.entry("BlueNorth",   new Pose2d(3.454, 7.622, Rotation2d.fromDegrees(0.0))),
        Map.entry("BlueCenter",  new Pose2d(3.454, 4.041, Rotation2d.fromDegrees(0.0))),
        Map.entry("BlueSouth",   new Pose2d(3.454, 0.460, Rotation2d.fromDegrees(0.0))),

        // Red North
        Map.entry("RedNorth",    new Pose2d(13.080, 7.622, Rotation2d.fromDegrees(180.0))),
        Map.entry("RedCenter",   new Pose2d(13.080, 4.041, Rotation2d.fromDegrees(180.0))),
        Map.entry("RedSouth",    new Pose2d(13.080, 0.460, Rotation2d.fromDegrees(180.0)))
    );

    public static final Map<String, Pose2d> AUTON_TARGET_POSES = Map.ofEntries(
        // Blue North
        Map.entry("BlueNorthAlpha",   new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(83.478))),
        Map.entry("BlueNorthBravo",   new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(119.775))),
        Map.entry("BlueNorthDefault", new Pose2d(2.938, 4.808, Rotation2d.fromDegrees(83.478))),
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
        Map.entry("RedSouthDefault",  new Pose2d(13.780, 3.250, Rotation2d.fromDegrees(-42.038)))
    );
    public static Pose2d getShootPosition() {
        var alliance = DriverStation.getAlliance();
        // TODO - if not set then PANIC
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return TargetPositions.RED_SHOOT;
        }

        return TargetPositions.BLUE_SHOOT;
    }

    public static Pose2d getActualStartingPos(String strategy) throws IllegalArgumentException {
        String position = strategy.replace("Alpha", "").replace("Bravo", "").replace("Default", "");
        Pose2d pose = AUTON_START_POSES.get(position);
        if (pose == null) {
            throw new IllegalArgumentException("No starting pose found for position: '" + position + "' from strategy: " + strategy);
        }

        return pose;
    }

    public static Pose2d getDesiredPose(String strategy) throws IllegalArgumentException {
        Pose2d pose = AUTON_TARGET_POSES.get(strategy);
        if (pose == null) {
            throw new IllegalArgumentException("No starting pose found for strategy: " + strategy);
        }
        return pose;
    }
}