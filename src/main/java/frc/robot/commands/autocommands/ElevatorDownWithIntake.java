package frc.robot.commands.autocommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.AlgaeIntake;
import frc.robot.subsystems.mechanisms.Elevator;

/**
 * Drives the elevator DOWN toward a target while running the intake.
 * - Only "finishes" once we actually moved down to/below the target (we must have started above it).
 * - After reaching the target, keeps the intake on for postDelaySec, then stops intake and elevator.
 */
public class ElevatorDownWithIntake extends Command {
  private final Elevator elevator;
  private final AlgaeIntake intake;

  private final DoubleSupplier goal;  // same units as elevator.getPosition()
  private final double downMag;       // magnitude; applied as negative
  private final double tolerance;     // directional tolerance
  private final double intakeSpeed;   // intake motor speed while running
  private final double postDelaySec;  // keep intake on after reaching goal

  private final Timer postTimer = new Timer();
  private boolean reachedGoal = false;
  private boolean startAboveGoal = false;

  public ElevatorDownWithIntake(
      Elevator elevator,
      AlgaeIntake intake,
      DoubleSupplier goal,
      double downMagnitude,
      double tolerance,
      double intakeSpeed,
      double postDelaySec) {

    this.elevator = elevator;
    this.intake = intake;
    this.goal = goal;
    this.downMag = Math.abs(downMagnitude);
    this.tolerance = tolerance;
    this.intakeSpeed = intakeSpeed;
    this.postDelaySec = postDelaySec;

    addRequirements(elevator, intake);
  }

  @Override
  public void initialize() {
    reachedGoal = false;
    postTimer.stop();
    postTimer.reset();

    double cur = elevator.getPosition();
    double tgt = goal.getAsDouble();
    startAboveGoal = cur > tgt;     // must start ABOVE the target for a down move

    intake.set(intakeSpeed);        // start intake immediately
    elevator.set(-downMag);         // start moving down

    System.out.printf(
        "[ElevatorDownWithIntake] START cur=%.3f tgt=%.3f tol=%.3f down=%.2f intake=%.2f post=%.1fs startAbove=%b%n",
        cur, tgt, tolerance, downMag, intakeSpeed, postDelaySec, startAboveGoal);
  }

  // directional "at-goal" check for DOWN moves
  private boolean atGoalDown(double cur, double tgt) {
    // considered done when we are at/below target, or just above within +tolerance
    return cur <= (tgt + tolerance);
  }

  @Override
  public void execute() {
    double cur = elevator.getPosition();
    double tgt = goal.getAsDouble();

    if (!reachedGoal) {
      // Only allow completion if we started above the goal; otherwise keep intake on but don't "reach"
      if (startAboveGoal && atGoalDown(cur, tgt)) {
        reachedGoal = true;
        elevator.set(0);         // stop elevator motion
        postTimer.restart();     // keep intake on during the post delay
        System.out.printf("[ElevatorDownWithIntake] Reached goal cur=%.3f tgt=%.3f -> starting post delay%n", cur, tgt);
      } else {
        // If we are still above target by more than tolerance, keep commanding down
        if (cur > tgt + tolerance) {
          elevator.set(-downMag);
        } else {
          // We are near/below target but didn't start above; don't drive down further
          elevator.set(0);
        }
      }
    }
    // Once reachedGoal == true, we just wait for post delay to elapse with intake running
  }

  @Override
  public void end(boolean interrupted) {
    elevator.set(0);
    intake.set(0);
    System.out.printf("[ElevatorDownWithIntake] END interrupted=%b cur=%.3f%n",
        interrupted, elevator.getPosition());
  }

  @Override
  public boolean isFinished() {
    // Finish only after we've actually reached the goal (from above) and the post delay elapsed
    return reachedGoal && postTimer.hasElapsed(postDelaySec);
  }
}
