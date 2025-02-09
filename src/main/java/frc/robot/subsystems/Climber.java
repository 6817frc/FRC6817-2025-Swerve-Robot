// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Climber extends SubsystemBase {
  // public final SparkMax m_climbFront; //TODO uncomment here to...
	// public final SparkMax m_climbBack;
  // public final SparkClosedLoopController frontClimbPID;
  // public final SparkClosedLoopController backClimbPID;
  // public final RelativeEncoder frontEncoder;
  // public final RelativeEncoder backEncoder; //TODO...here
  // private final DigitalInput limitSwitch = new DigitalInput(1);
  private double frontEncoderOffset;
  private double backEncoderOffset;
  private double realMotorPos;

  /** Creates a new Climber. */
  public Climber() {
    // m_climbFront = new SparkMax(Ports.CAN.ClimbFront, MotorType.kBrushless); //TODO uncomment here to...
    // m_climbBack = new SparkMax(Ports.CAN.ClimbBack, MotorType.kBrushless);
    // m_climbFront.setIdleMode(IdleMode.kBrake);
    // m_climbBack.setIdleMode(IdleMode.kBrake);
    // frontClimbPID = m_climbFront.getPIDController();
    // backClimbPID = m_climbBack.getPIDController();
    // frontClimbPID.setOutputRange(-0.5, 0.3);
    // backClimbPID.setOutputRange(-0.5, 0.3);
    double value = SmartDashboard.getNumber("climbPValue", 0.05);
    SmartDashboard.putNumber("climbPValue", value);
    // frontEncoder = m_climbFront.getEncoder();
    // backEncoder = m_climbBack.getEncoder();
    frontEncoderOffset = 0;
    backEncoderOffset = 0;
    // m_climbBack.follow(m_climbFront); // TODO...here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
