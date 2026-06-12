package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;

public class RotateAroundPointCommand extends Command {
    Drive swerve;
    private Rotation2d rotation;
    private Pose2d targetPose;
    private Pose2d currentPose;
    private Translation2d center;
    
    
    public RotateAroundPointCommand(Drive swerve, Rotation2d rotation, Translation2d center, Pose2d currentPose) {
        this.swerve = swerve;
        this.rotation = rotation;
        this.center = center;
        this.currentPose = currentPose;
    }

    @Override
    public void initialize() {
        targetPose = currentPose.rotateAround(center, rotation);
        swerve.setTargetPosition(targetPose);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (Math.abs(targetPose.getRotation().minus(RobotState.getInstance().getEstimatedPose().getRotation()).getDegrees()) < 2 
        && targetPose.getTranslation().getDistance(RobotState.getInstance().getEstimatedPose().getTranslation()) < 0.04) {
            return true;
        } else {
            return false;
        }
    }

}
