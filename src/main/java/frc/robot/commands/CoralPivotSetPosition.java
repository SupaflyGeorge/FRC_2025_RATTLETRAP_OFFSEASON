package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.CoralPivot;

/** Drives pivot toward a target rotations (open-loop) and stops within tolerance. */
public class CoralPivotSetPosition extends Command {
  private final CoralPivot pivot;
  private final double targetRot;
  private final double upSpeed;     // +speed when target > current
  private final double downSpeed;   // -speed when target < current
  private final double tolerance;

  public CoralPivotSetPosition(CoralPivot pivot, double targetRot, double upSpeed, double downSpeed, double tolerance) {
    this.pivot = pivot;
    this.targetRot = targetRot;
    this.upSpeed = Math.abs(upSpeed);
    this.downSpeed = -Math.abs(downSpeed);
    this.tolerance = Math.abs(tolerance);
    addRequirements(pivot);
  }

  @Override public void execute() {
    double cur = pivot.getPosition();
    double err = targetRot - cur;
    if (Math.abs(err) <= tolerance) {
      pivot.stop();
    } else if (err > 0) {
      pivot.set(upSpeed);
    } else {
      pivot.set(downSpeed);
    }
  }

  @Override public void end(boolean interrupted) {
     pivot.stop(); }
  @Override public boolean isFinished() {
     return Math.abs(targetRot - pivot.getPosition()) <= tolerance; }
}
