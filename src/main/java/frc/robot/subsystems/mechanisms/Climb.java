// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final SparkMax climb = new SparkMax(13, MotorType.kBrushless);
 
  
  private final RelativeEncoder climbMotorEncoder = climb.getEncoder();
  

  /** Creates a new Arm. */
  public Climb() {

    final double elevatorMotorRevToDegreesOfElevator = 360.0 / (64 * 5);
    climbMotorEncoder.setPosition(elevatorMotorRevToDegreesOfElevator);
    

    final double elevatorWhenLowered = 0;
    climbMotorEncoder.setPosition(elevatorWhenLowered);
  
  }

  public void set(double speed) {
   climb.set(speed);
    
  }

  public double getPosition() {
    return (climbMotorEncoder.getPosition()) / 2;
  }

  public double getVelocity() {
    return (climbMotorEncoder.getVelocity()) / 10;
  }

  public void configureShuffleboard() {
    Shuffleboard.getTab("Debug (Sensors)").addDouble("Arm Position (Degrees)", () -> getPosition());
  }

  public void resetEncoders() {
    climbMotorEncoder.setPosition(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
