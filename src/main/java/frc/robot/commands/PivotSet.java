package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Pivot;

public class PivotSet extends Command {
  private final Pivot pivotSubsystem;
  private final double speed;

  public PivotSet(Pivot pivotSubsystem, double speed) {
    this.pivotSubsystem = pivotSubsystem;
    this.speed = speed;
    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {
    // optional: start moving immediately
    pivotSubsystem.set(speed);
  }

  @Override
  public void execute() {
    // hold the set speed while the command is scheduled
    pivotSubsystem.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.set(0.0);
  }

  
  @Override
  public boolean isFinished() {
    return false; // runs until interrupted (e.g., whileTrue binding)
  }
}
