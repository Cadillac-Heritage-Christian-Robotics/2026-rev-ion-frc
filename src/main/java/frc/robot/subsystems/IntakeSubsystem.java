// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
  // Initialize intake SPARK. We will use open loop control for this. 
  private SparkFlex intakeMotor =
     new SparkFlex(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
  
// Uncomment and comment/delete later the SparkFlex command then deploy the  
// private TalonFX intakeMotor =
  //    new TalonFX(IntakeSubsytemConstants.kIntakeMotorCanId);

  private SparkFlex slapMotor =
   new SparkFlex(IntakeSubsystemConstants.kSlapMotorCanId, MotorType.kBrushless);

  private PIDController slapMotorPID = new PIDController(10, 0, 0);
  // Initialize conveyor SPARK. We will use open loop control for this.
  //  private TalonFX conveyorMotor =
  //     new TalonFX(20); 
    private TalonFX conveyorMotor =
     new TalonFX(IntakeSubsystemConstants.kConveyorMotorCanId);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    intakeMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    slapMotor.configure(
      Configs.IntakeSubsystem.slapConfig.apply(Configs.IntakeSubsystem.slapEncoderConfig),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    slapMotor.getEncoder().setPosition(0.33);

    conveyorMotor.getConfigurator().apply(Configs.IntakeSubsystem.conveyorConfig);

    System.out.println("---> IntakeSubsystem initialized");
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  /** Set the conveyor motor power in the range of [-1, 1]. */
   public void setConveyorPower(double power) {
     conveyorMotor.set(power);
   }

  private void setSlapPosition(double position) {
    SmartDashboard.putNumber("Slap | Position", position);

    slapMotorPID.setSetpoint(position);
  }

  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kIntake);
          this.setConveyorPower(ConveyorSetpoints.kExtake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Intaking");
  }

  public Command runSlapUpCommand() {
    return new InstantCommand(() -> {this.setSlapPosition(0.3);});
    }

   public Command runSlapDownCommand() {
    return new InstantCommand(() -> {this.setSlapPosition(0.001);});
    }

  /**
   * Command to reverse the intake motor and coveyor motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          this.setConveyorPower(ConveyorSetpoints.kIntake);
        }, () -> {
          this.setIntakePower(0.0);
          this.setConveyorPower(0.0);
        }).withName("Extaking");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Intake | Intake | Power", intakeMotor.get());
    SmartDashboard.putNumber("Intake | Slap | Position", slapMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake | Conveyor | Applied Output", conveyorMotor.getDutyCycle().getValueAsDouble());

    slapMotor.setVoltage(MathUtil.clamp(slapMotorPID.calculate(slapMotor.getEncoder().getPosition()), -12, 12));
  }
}
