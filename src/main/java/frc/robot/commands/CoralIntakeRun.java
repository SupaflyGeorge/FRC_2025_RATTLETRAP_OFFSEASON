package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.CoralIntake;

/** Runs the coral intake at a fixed speed while held. */
public class CoralIntakeRun extends Command {
  private final CoralIntake intake;
  private final double speed;

  public CoralIntakeRun(CoralIntake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override public void initialize() {
    System.out.println("[CoralIntakeRun] start speed=" + speed);
    intake.setPercent(speed);
  }

  @Override public void end(boolean interrupted) {
    intake.stop();
    System.out.println("[CoralIntakeRun] end interrupted=" + interrupted);
  }

  @Override public boolean isFinished() { return false; }
}

