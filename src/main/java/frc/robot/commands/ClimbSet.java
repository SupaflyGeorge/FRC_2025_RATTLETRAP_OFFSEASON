package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Climb;

public class ClimbSet extends Command {
    public final Climb climbSubsystem;
    public final double speed;

    //Creates new Feed
    public ClimbSet(Climb climbSubsystem, double speed) {
        this.climbSubsystem = climbSubsystem;
        this.speed = speed;

        addRequirements(climbSubsystem);
    }

    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        climbSubsystem.set(-speed);
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        climbSubsystem.set(0);
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }
}
