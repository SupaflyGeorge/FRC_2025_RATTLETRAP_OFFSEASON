package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.CoralPivot;

/** Open-loop pivot command: runs at a fixed speed until interrupted. */
public class CoralPivotSet extends Command {
  private final CoralPivot pivot;
  private final double speed;

  public CoralPivotSet(CoralPivot pivot, double speed) {
    this.pivot = pivot;
    this.speed = speed;
    addRequirements(pivot);
  }

  @Override public void initialize() {
     pivot.set(speed); }

  @Override public void execute() {

   }
  @Override public void end(boolean interrupted) { 
    pivot.stop(); }

  @Override public boolean isFinished() {
     return false; }
}
