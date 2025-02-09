// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.commands.intake.CoralIntakeMovement;

public class CoralIntake extends SubsystemBase {
  /** Creates a new Intake. */

  //These are the motors used in the subsystem.
  public final AddressableLED m_LED;
  public final AddressableLEDBuffer m_LedBuffer;
  public final SparkMax m_intakeWheels;
	public final SparkMax m_intakeArm;
  public final SparkClosedLoopController armPID;
  public final RelativeEncoder armEncoder;
  private double encoderOffset;
  private double realMotorPos;

  public CoralIntake() {
    m_LED = new AddressableLED(Ports.PWM.LED_STRIP);
    m_LedBuffer = new AddressableLEDBuffer(8);
    m_LED.setLength(m_LedBuffer.getLength());
    m_LED.setData(m_LedBuffer);
    m_LED.start();

    m_intakeWheels = new SparkMax(Ports.CAN.Intake, MotorType.kBrushless); //TODO update for new motors
    m_intakeWheels.setIdleMode(SparkBaseConfig.IdleMode.kBrake); //TODO update for new motors
    m_intakeArm = new SparkMax(Ports.CAN.Arm, MotorType.kBrushed); //TODO update for new motors
    m_intakeArm.setInverted(true); //TODO check whether this needs to be inverted
    m_intakeArm.setIdleMode(IdleMode.kBrake); //TODO update for new motors
    armPID = m_intakeArm.getClosedLoopController(); //TODO update for new motors
    armPID.setOutputRange(-0.5, 0.25);

    double value = SmartDashboard.getNumber("PValue", 3);
    SmartDashboard.putNumber("PValue", value);
    armEncoder = m_intakeArm.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 2048);
    encoderOffset = 0; 
  }
    
    //sets the offset as the current value of the encoder
    public void resetArmEncoder() {
      encoderOffset = armEncoder.getPosition();
    }

    /*subtracts the offset from the current position 
    and sets that as the new motor position*/
    public double realMotorPos() {
      realMotorPos = armEncoder.getPosition() - encoderOffset;
      return realMotorPos;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}