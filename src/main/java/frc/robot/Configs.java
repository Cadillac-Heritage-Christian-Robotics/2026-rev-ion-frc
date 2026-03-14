package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  }
  
  public static final class IntakeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final TalonFXConfiguration conveyorConfig = new TalonFXConfiguration();

    static {
      // Configure basic settings of the intake motor
      intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.5) // TODO tune this!
        .smartCurrentLimit(40); // TODO tune this!

      // Configure basic settings of the conveyor motor (TalonFX)
      conveyorConfig.CurrentLimits.StatorCurrentLimit = 40; // TODO - tune this
      conveyorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      conveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO adjust if needed!
    }
  }

    public static final class ShooterSubsystem {
    public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();
        private static final double nominalVoltage = 0;
        
        static {
          // Configure basic setting of the flywheel motors
          flywheelConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .closedLoopRampRate(1.0)
            .openLoopRampRate(1.0)
            .smartCurrentLimit(80);
    
          /*
           * Configure the closed loop controller. We want to make sure we set the
           * feedback sensor as the primary encoder.
           */
          flywheelConfig
            .closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              // Set PID values for position control
              .p(0.0002)
              .outputRange(-1, 1);
    
          flywheelConfig.closedLoop
            .maxMotion
              // Set MAXMotion parameters for MAXMotion Velocity control
              .cruiseVelocity(5000)
              .maxAcceleration(10000)
              .allowedProfileError(1);
    
              // Constants.NeoMotorConstants.kVortexKv is in rpm/V. feedforward.kV is in V/rpm sort we take
              // the reciprocol.
              flywheelConfig.closedLoop
                .feedForward.kV(nominalVoltage / Constants.NeoMotorConstants.kVortexKv);

      // Configure the follower flywheel motor to follow the main flywheel motor
      flywheelFollowerConfig.apply(flywheelConfig)
        .follow(Constants.ShooterSubsystemConstants.kFlywheelMotorCanId, true);

      // Configure basic setting of the feeder motor
      feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(60);
    }
  }
}