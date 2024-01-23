package frc.robot.commands;

import static frc.robot.Constants.limelightName;
import static frc.robot.Constants.rotation_P;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
    public class AprilTagAimCommand extends Command {

    private String target;
    private double tx = LimelightHelpers.getTX(limelightName);
    private double steeringAdjust; 

    private final Drivetrain drivetrain;
    public AprilTagAimCommand(Drivetrain drivetrain, String target) {
        this.drivetrain = drivetrain;
        this.target = target;
        addRequirements(drivetrain);
    }
    @Override
    public void execute() {
        switch (target) {
            case "speaker":
                //setPipelineIndex(limelightName, 0);
                break;
            case "amp":
                //setPipelineIndex(limelightName, 1);
                break;
            case "source":
                //setPipelineIndex(limelightName, 2);
                break;
        }
        steeringAdjust = rotation_P * tx;
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            steeringAdjust,
            Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())));
    }
    @Override
    public void end(boolean interrupted) {
    }
}