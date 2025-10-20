package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;

public class ElevatorUp extends Command {
  private final Elevator elevatorSubsystem;
  private final double speed;

  public ElevatorUp(Elevator elevatorSubsystem, double speed) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.speed = speed;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    // HARD disable PID so manual is truly open-loop
    elevatorSubsystem.disableClosedLoop();
  }

  @Override
  public void execute() {
    elevatorSubsystem.set(speed); // pure open-loop up (soft limits still apply)
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.set(0.0);   // stop motors; LEAVE PID OFF
  }

  @Override
  public boolean isFinished() { return false; }
}
