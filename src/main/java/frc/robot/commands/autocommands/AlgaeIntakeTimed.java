package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.AlgaeIntake;

/**
 * Runs the algae intake motor at a given speed for a fixed duration,
 * then stops it.
 */
public class AlgaeIntakeTimed extends Command {
  private final AlgaeIntake intake;
  private final double speed;
  private final double durationSec;
  private final Timer timer = new Timer();

  public AlgaeIntakeTimed(AlgaeIntake intake, double speed, double durationSec) {
    this.intake = intake;
    this.speed = speed;
    this.durationSec = durationSec;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.set(speed);
    System.out.printf("[AlgaeIntakeTimed] START speed=%.2f duration=%.1fs%n", speed, durationSec);
  }

  @Override
  public void execute() {
    // nothing else needed, motor is already running
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.printf("[AlgaeIntakeTimed] END interrupted=%b%n", interrupted);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(durationSec);
  }
}
