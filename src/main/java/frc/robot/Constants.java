// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Fuel Intake
  public static final class IntakeSubsystemConstants {
    public static final int kIntakeMotorCanId = 5;    // SPARK Flex CAN ID
    public static final int kSlapMotorCanId = 6;
    public static final int kConveyorMotorCanId = 20;  // SPARK Flex CAN ID

    public static final class IntakeSetpoints {
      public static final double kIntake = 0.6;
      public static final double kExtake = -0.6;
    }

    public static final class ArmSetpoints {
      public static final double kLevel1 = 0.1;
      public static final double kLevel2 = -0.1;
    }

    public static final class ConveyorSetpoints {
      public static final double kIntake = 0.7;
      public static final double kExtake = -0.7;
    }
  }

    public static final class ShooterSubsystemConstants {
     public static final int kFeederMotorCanId = 22;    // SPARK Flex CAN ID
     public static final int kFlywheelMotorCanId = 21;  // SPARK Flex CAN ID (Right)
     public static final int kFlywheelFollowerMotorCanId = 23;  // SPARK Flex CAN ID (Left)

    public static final class FeederSetpoints {
      public static final double kFeed = 0.95;
    }

    public static final class FlywheelSetpoints {
      public static final double kShootRpm = 5000;
      public static final double kVelocityTolerance = 100;
    }
  }

    public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
    public static final double kVortexKv = 565;   // rpm/V
  }

  public static final class TargetPositions {
    // Alliance shoot positions
    public static final Pose2d BLUE_SHOOT = new Pose2d(3.0, 4.1, new Rotation2d());
    public static final Pose2d RED_SHOOT  = new Pose2d(13.5, 4.1, new Rotation2d());

    // Reload poles
    public static final Pose2d NORTH_POLE = new Pose2d(8.23, 7.2, new Rotation2d());
    public static final Pose2d SOUTH_POLE = new Pose2d(8.23, 1.0, new Rotation2d());
  }

   /** OIConstants ****/
   public static final class OIConstants {
     public static final int kDriverControllerPort = 0;   
     public static final double kDriveDeadband = 0.1;
     public static final double kTriggerButtonThreshold = 0.2;
   }

}

