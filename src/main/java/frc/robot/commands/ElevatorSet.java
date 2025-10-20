// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;;

public class ElevatorSet extends Command {
  private final Elevator elevatorSubsystem;
  private final DoubleSupplier speedSupplier;
  
  /** Creates a new ArmSet. */
  public ElevatorSet(Elevator elevatorSubsystem, DoubleSupplier speedSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.speedSupplier = () -> -speedSupplier.getAsDouble();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cmd = speedSupplier.getAsDouble();
    if (Math.abs(cmd) < 0.05) cmd = 0;
    elevatorSubsystem.set(cmd); // soft limits inside Elevator clamp motion
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}