// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Local Imports
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {

  // TODO: Move these to Constants.java
  private static final int kFlywheelMotorCanId = 21;
  private static final int kFlywheelFollowerMotorCanId = 23;
  private static final int kFeederMotorCanId = 22;

  // TODO: Phoenix6 velocity is in rotations per second - tune this value!
  // Original was 5000 RPM ≈ 83.3 RPS
  private static final double kShootVelocityRPS = 83.3;
  private static final double kVelocityTolerance = 2.0; // RPS

  // Initialize flywheel TalonFXs. We will use MotionMagic velocity control for
  // the flywheel to allow for smooth acceleration/deceleration to setpoint.
  private final TalonFX flywheelMotor = new TalonFX(kFlywheelMotorCanId);
  private final TalonFX flywheelFollowerMotor = new TalonFX(kFlywheelFollowerMotorCanId);

  // Initialize feeder TalonFX. We will use open loop (DutyCycle) control for this.
  private final TalonFX feederMotor = new TalonFX(kFeederMotorCanId);

  // Control requests - reuse these objects to avoid garbage collection
  private final MotionMagicVelocityVoltage flywheelVelocityRequest = new MotionMagicVelocityVoltage(0);
  private final DutyCycleOut feederDutyCycleRequest = new DutyCycleOut(0);

  // Member variables for subsystem state management
  private double flywheelTargetVelocity = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Configure flywheel motor
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    // TODO: Tune these PID/MotionMagic values for your robot!
    flywheelConfig.Slot0.kP = 0.1;
    flywheelConfig.Slot0.kI = 0.0;
    flywheelConfig.Slot0.kD = 0.0;
    flywheelConfig.Slot0.kV = 0.12; // Feedforward: ~1/kMaxRPS
    flywheelConfig.MotionMagic.MotionMagicAcceleration = 400; // RPS/s
    flywheelConfig.MotionMagic.MotionMagicJerk = 4000;        // RPS/s^2
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    // Configure follower - set to oppose leader direction if motors are mounted mirrored
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    flywheelFollowerMotor.getConfigurator().apply(followerConfig);

    // TODO: Use Aligned if the motors are mounted the same direction, 
    // TODO: Opposed if they're mirrored facing each other
    flywheelFollowerMotor.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Configure feeder motor
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederMotor.getConfigurator().apply(feederConfig);

    // Zero flywheel encoder on initialization
    flywheelMotor.setPosition(0);

    System.out.println("---> ShooterSubsystem initialized");
  }

  private boolean isFlywheelAt(double velocityRPS) {
    return MathUtil.isNear(
        flywheelMotor.getVelocity().getValueAsDouble(),
        velocityRPS,
        kVelocityTolerance);
  }

  /**
   * Trigger: Is the flywheel spinning at the required velocity?
   */
  public final Trigger isFlywheelSpinning = new Trigger(
      () -> isFlywheelAt(kShootVelocityRPS) || flywheelMotor.getVelocity().getValueAsDouble() > kShootVelocityRPS);

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> isFlywheelAt(-kShootVelocityRPS) || flywheelMotor.getVelocity().getValueAsDouble() < -kShootVelocityRPS);

  /**
   * Trigger: Is the flywheel stopped?
   */
  public final Trigger isFlywheelStopped = new Trigger(() -> isFlywheelAt(0));

  /**
   * Drive the flywheels to their set velocity using MotionMagic velocity control,
   * which allows for smooth acceleration and deceleration to the setpoint.
   */
  private void setFlywheelVelocity(double velocityRPS) {
    flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(velocityRPS));
    flywheelTargetVelocity = velocityRPS;
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power) {
    feederMotor.setControl(feederDutyCycleRequest.withOutput(power));
  }

  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the
   * button is released, the motors will stop.
   */
  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> this.setFlywheelVelocity(kShootVelocityRPS),
        () -> this.setFlywheelVelocity(0.0))
        .withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is
   * interrupted, e.g. the button is released, the motors will stop.
   */
  public Command runFeederCommand() {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(kShootVelocityRPS);
          this.setFeederPower(0.8); // TODO: Move to constants
        },
        () -> {
          this.setFlywheelVelocity(0.0);
          this.setFeederPower(0.0);
        }).withName("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The flywheel starts spinning up and when
   * it reaches the desired speed it starts the feeder.
   */
  public Command runShooterCommand() {
    return this.startEnd(
        () -> this.setFlywheelVelocity(kShootVelocityRPS),
        () -> flywheelMotor.stopMotor())
        .until(isFlywheelSpinning)
        .andThen(
            this.startEnd(
                () -> {
                  this.setFlywheelVelocity(kShootVelocityRPS);
                  this.setFeederPower(0.8); // TODO: Move to constants
                },
                () -> {
                  flywheelMotor.stopMotor();
                  feederMotor.stopMotor();
                }))
        .withName("Shooting");
  }

  @Override
  public void periodic() {
    // Limelight hello world!s
    Boolean found_target = LimelightHelpers.getTV("");
    SmartDashboard.putBoolean(("LL Has Target In Sights - Fire away"), found_target);
    // SmartDashboard.putBoolean("LL Has Target", LimelightHelpers.getTV(""));
    SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("LL TY", LimelightHelpers.getTY(""));

    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Feeder | Applied Output",
        feederMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output",
        flywheelMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Shooter | Flywheel | Current",
        flywheelMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output",
        flywheelFollowerMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Current",
        flywheelFollowerMotor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Shooter | Flywheel | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Velocity",
        flywheelMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }

  //  @Override
  //  public void simulationPeriodic() {
  //   // TODO Auto-generated method stub
  //   // super.simulationPeriodic();
  //   // System.out.println("Running simulation periodic");
  //   Boolean found_target = LimelightHelpers.getTV("limelight");
  //   // System.out.println(found_target);
  //   SmartDashboard.putBoolean(("LL Has Target In Sights - Fire away"), found_target);
  //   // SmartDashboard.putBoolean("LL Has Target", LimelightHelpers.getTV(""));
  //   SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""));
  //   SmartDashboard.putNumber("LL TY", LimelightHelpers.getTY(""));

  // }
}