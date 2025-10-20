package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.CoralIntake;

public class CoralIntakeVoltageTest extends Command {
  private final CoralIntake intake;
  private final double volts, seconds;
  private final Timer timer = new Timer();

  public CoralIntakeVoltageTest(CoralIntake intake, double volts, double seconds) {
    this.intake = intake; this.volts = volts; this.seconds = seconds;
    addRequirements(intake);
  }

  @Override public void initialize() {
    System.out.printf("[IntakeVTest] start V=%.1f for %.1fs%n", volts, seconds);
    timer.reset(); timer.start();
    intake.setVolts(volts);
  }
  @Override public void end(boolean interrupted) {
    intake.stop();
    System.out.println("[IntakeVTest] end interrupted=" + interrupted);
  }
  @Override public boolean isFinished() { return timer.hasElapsed(seconds); }
}
