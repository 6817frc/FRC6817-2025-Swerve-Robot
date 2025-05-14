// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeL2 extends Command {

  public CoralIntake intake;
  public AddressableLED m_LED;
  public AddressableLEDBuffer m_LedBuffer;
  public boolean inPosition;
  public double deadband;
  private int timer;
  public int testNumber;

  public IntakeL2(CoralIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }  

  public void isPosition() {
    if (intake.armEncoder.getPosition() <= 0.33 && intake.wristEncoder.getPosition() <= 0.31) {
      inPosition = true;
    }
  }

  public void dropCoral() {
    if (inPosition == true && timer < 2) {
      intake.m_intakeWheels.set(-0.2);
      timer++;
    } else {
      intake.m_intakeWheels.set(0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inPosition = false;
    deadband = 0.01;
    timer = 0;
    testNumber = 1;
    intake.armL2();
  }

  @Override
  public void execute() {
    testNumber = 2;
    isPosition();
    dropCoral();
  }

  @Override
  public boolean isFinished() {
    if (inPosition == true) {
      testNumber = 3;
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      testNumber = 4;
      intake.stopIntake();
      intake.stopWrist();
    } 
  }
}
