package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Pivot;

public class PivotToPosition extends Command {
  private final Pivot pivot;
  private final double speed;
  private final double stopPosition;

  public PivotToPosition(Pivot pivot, double speed, double stopPosition) {
    this.pivot = pivot;
    this.speed = speed;
    this.stopPosition = stopPosition;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.set(0);
  }

  @Override
  public boolean isFinished() {
    // identical logic to your inline .until(...)
    return pivot.getPosition() > stopPosition;

  }
}
