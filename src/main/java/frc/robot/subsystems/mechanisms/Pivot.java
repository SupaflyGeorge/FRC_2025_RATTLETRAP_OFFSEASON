// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final SparkMax pivot = new SparkMax(11, MotorType.kBrushless);
 
  
  private final RelativeEncoder pivotMotorEncoder = pivot.getEncoder();
  

  /** Creates a new Arm. */
  public Pivot() {

    final double elevatorMotorRevToDegreesOfElevator = 360.0 / (64 * 5);
    pivotMotorEncoder.setPosition(elevatorMotorRevToDegreesOfElevator);
    

    final double elevatorWhenLowered = 0;
    pivotMotorEncoder.setPosition(elevatorWhenLowered);
  
  }

  public void set(double speed) {
   pivot.set(speed);
    
  }

  public double getPosition() {
    return (pivotMotorEncoder.getPosition()) / 2;
  }

  public double getVelocity() {
    return (pivotMotorEncoder.getVelocity()) / 10;
  }

  public void configureShuffleboard() {
    Shuffleboard.getTab("Debug (Sensors)").addDouble("Arm Position (Degrees)", () -> getPosition());
  }

  public void resetEncoders() {
    pivotMotorEncoder.setPosition(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
