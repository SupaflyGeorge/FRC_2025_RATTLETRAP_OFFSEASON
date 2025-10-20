package frc.robot.commands.autocommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;

public class ElevatorGoTo extends Command {
  private final Elevator elevator;
  private final DoubleSupplier goal;
  private final double upMag;    // speed magnitude for UP  (applied as +)
  private final double downMag;  // speed magnitude for DOWN (applied as -)
  private final double tol;
  private final BooleanSupplier upBlocked; // may be () -> false

  public ElevatorGoTo(Elevator elevator,
                      DoubleSupplier goal,
                      double upMagnitude,
                      double downMagnitude,
                      double tolerance) {
    this(elevator, goal, upMagnitude, downMagnitude, tolerance, () -> false);
  }

  public ElevatorGoTo(Elevator elevator,
                      DoubleSupplier goal,
                      double upMagnitude,
                      double downMagnitude,
                      double tolerance,
                      BooleanSupplier upBlocked) {
    this.elevator = elevator;
    this.goal = goal;
    this.upMag = Math.abs(upMagnitude);
    this.downMag = Math.abs(downMagnitude);
    this.tol = tolerance;
    this.upBlocked = upBlocked;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    System.out.printf("[ElevatorGoTo] START tgt=%.3f tol=%.3f%n", goal.getAsDouble(), tol);
  }

  @Override
  public void execute() {
    double cur = elevator.getPosition();
    double tgt = goal.getAsDouble();
    double err = tgt - cur;

    if (Math.abs(err) <= tol) {
      elevator.set(0);
      return;
    }

    if (err > 0) {
      // need to go UP
      if (upBlocked.getAsBoolean()) {
        elevator.set(0);
        System.out.printf("[ElevatorGoTo] UP BLOCKED cur=%.3f tgt=%.3f%n", cur, tgt);
      } else {
        elevator.set(+upMag);
        // System.out.printf("[ElevatorGoTo] UP cur=%.3f tgt=%.3f spd=+%.2f%n", cur, tgt, upMag);
      }
    } else {
      // need to go DOWN (never blocked by top latch)
      elevator.set(-downMag);
      // System.out.printf("[ElevatorGoTo] DOWN cur=%.3f tgt=%.3f spd=-%.2f%n", cur, tgt, downMag);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.set(0);
    System.out.printf("[ElevatorGoTo] END interrupted=%b cur=%.3f%n", interrupted, elevator.getPosition());
  }

  @Override
  public boolean isFinished() {
    double cur = elevator.getPosition();
    double tgt = goal.getAsDouble();
    boolean atTarget = Math.abs(tgt - cur) <= tol;
    boolean blockedUp = (tgt > cur) && upBlocked.getAsBoolean();
    return atTarget || blockedUp;
  }
}
