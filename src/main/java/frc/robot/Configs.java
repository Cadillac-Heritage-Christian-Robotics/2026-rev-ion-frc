package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
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
    public static final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

    static {
      // Configure basic settings of the intake motor
      intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(40);

      // Configure basic settings of the conveyor motor
      conveyorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(40);
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


  // public static final class CoralSubsystem {
  //   public static final SparkMaxConfig armConfig = new SparkMaxConfig();
  //   public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
  //   public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

  //   static {
  //     // Configure basic settings of the arm motor
  //     armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

  //     /*
  //      * Configure the closed loop controller. We want to make sure we set the
  //      * feedback sensor as the primary encoder.
  //      */
  //     armConfig
  //         .closedLoop
  //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //         // Set PID values for position control
  //         .p(0.1)
  //         .outputRange(-1, 1)
  //         .maxMotion
  //         // Set MAXMotion parameters for position control
  //         .cruiseVelocity(2000)
  //         .maxAcceleration(10000)
  //         .allowedProfileError(0.25);

  //     // Configure basic settings of the elevator motor
  //     elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

  //     /*
  //      * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
  //      * will prevent any actuation of the elevator in the reverse direction if the limit switch is
  //      * pressed.
  //      */
  //     elevatorConfig
  //         .limitSwitch
  //         .reverseLimitSwitchEnabled(true)
  //         .reverseLimitSwitchType(Type.kNormallyOpen);

  //     /*
  //      * Configure the closed loop controller. We want to make sure we set the
  //      * feedback sensor as the primary encoder.
  //      */
  //     elevatorConfig
  //         .closedLoop
  //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //         // Set PID values for position control
  //         .p(0.1)
  //         .outputRange(-1, 1)
  //         .maxMotion
  //         // Set MAXMotion parameters for position control
  //         .cruiseVelocity(4200)
  //         .maxAcceleration(6000)
  //         .allowedProfileError(0.5);

  //     // Configure basic settings of the intake motor
  //     intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
  //   }
  // }

  // public static final class AlgaeSubsystem {
  //   public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
  //   public static final SparkFlexConfig armConfig = new SparkFlexConfig();

  //   static {
  //     // Configure basic setting of the arm motor
  //     armConfig.smartCurrentLimit(40);

  //     /*
  //      * Configure the closed loop controller. We want to make sure we set the
  //      * feedback sensor as the primary encoder.
  //      */
  //     armConfig
  //         .closedLoop
  //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //         // Set PID values for position control. We don't need to pass a closed
  //         // loop slot, as it will default to slot 0.
  //         .p(0.1)
  //         .outputRange(-0.5, 0.5);

  //     // Configure basic settings of the intake motor
  //     intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
  //   }
  // }
}