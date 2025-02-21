// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.commands.intake.CoralIntakeMovement;
import com.revrobotics.spark.SparkBase.ResetMode;

public class CoralIntake extends SubsystemBase {
  /** Creates a new Intake. */

  //These are the motors used in the subsystem.
  public final AddressableLED m_LED;
  public final AddressableLEDBuffer m_LedBuffer;
  public final SparkMax m_intakeWheels;
  public final SparkMax m_intakeWrist;
	public final SparkMax m_intakeArm1;
  public final SparkMax m_intakeArm2;
  // public final SparkClosedLoopController armPID;
  public final RelativeEncoder armEncoder;
  private double encoderOffset;
  private double realMotorPos;

  public CoralIntake() {
    m_LED = new AddressableLED(Ports.PWM.LED_STRIP);
    m_LedBuffer = new AddressableLEDBuffer(8);
    m_LED.setLength(m_LedBuffer.getLength());
    m_LED.setData(m_LedBuffer);
    m_LED.start();

    //This sets the configuration for the motor controlling the wheels of the intake subsystem
    m_intakeWheels = new SparkMax(Ports.CAN.Intake, MotorType.kBrushless); //TODO update for new motors
    SparkMaxConfig wheelConfig = new SparkMaxConfig();
    wheelConfig.inverted(false).idleMode(IdleMode.kBrake); //TODO update for new motors

    //This sets the config for the motor controlling the additional arm movement
    m_intakeWrist = new SparkMax(Ports.CAN.Wrist, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig.inverted(false).idleMode(IdleMode.kBrake);

    //This sets the configuration for one motor controlling the ovrall arm movement
    m_intakeArm1 = new SparkMax(Ports.CAN.Arm1, MotorType.kBrushed); //TODO update for new motors
    SparkMaxConfig arm1Config = new SparkMaxConfig();
    arm1Config.inverted(true).idleMode(IdleMode.kBrake); //TODO update for new motors
    arm1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0); //TODO update for new motors
    
    //This sets the configuration for other motor controlling the ovrall arm movement
    m_intakeArm2 = new SparkMax(Ports.CAN.Arm2, MotorType.kBrushless);
    SparkMaxConfig arm2Config = new SparkMaxConfig();
    arm2Config.inverted(false).idleMode(IdleMode.kBrake);

    // armPID.setOutputRange(-0.5, 0.25);  //TODO doesn't work anymore so find out how to limit motor output
    m_intakeWheels.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeArm1.configure(arm1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_intakeArm2.configure(arm2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    double value = SmartDashboard.getNumber("PValue", 3);
    SmartDashboard.putNumber("PValue", value);
    armEncoder = m_intakeArm1.getEncoder();
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