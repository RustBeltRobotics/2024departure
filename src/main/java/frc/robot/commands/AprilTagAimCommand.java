package frc.robot.commands;

import static frc.robot.Constants.limelightName;
import static frc.robot.Constants.speedLimit;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class AprilTagAimCommand extends Command {
    private double tx;
    private String target;
    private double sightedTID;
    private double targetTID;
    private double targetTID2 = -1;
    private double steeringAdjust; 
    private DoubleSupplier stickX;
    private DoubleSupplier stickY;
    private Optional<Alliance> alliance = DriverStation.getAlliance();

    private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    private GenericEntry kS =
      tab.add("kS", 0.1)
         .getEntry();
    private GenericEntry kV =
      tab.add("kV", 1.0)
         .getEntry();
    private GenericEntry kA =
      tab.add("kA", 0.0)
         .getEntry();



    private final Drivetrain drivetrain;
    public AprilTagAimCommand(Drivetrain drivetrain, String target, DoubleSupplier stickX, DoubleSupplier stickY) {
        this.drivetrain = drivetrain;
        this.target = target;
        this.stickX = stickX;
        this.stickY = stickY;
        addRequirements(drivetrain);
    }
    private final PIDController steerPID = new PIDController(.13, .13, .01);
    
    
    @Override
    public void execute() {
        System.out.println("execute start");
        final SimpleMotorFeedforward steerFeedforward = new SimpleMotorFeedforward(kS.getDouble(0.0), kV.getDouble(0.0), kA.getDouble(0.0));
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                switch (target) {
                case "speaker":
                    targetTID = 3;
                    targetTID2 = 4;
                    break;
                case "amp":
                    targetTID = 5;
                    break;
                case "source":
                    targetTID = 9;
                    targetTID2 = 10;                    
                    break;
                }
            }
        } else {//its blue
                switch (target) {
                case "speaker":
                    targetTID = 7;
                    targetTID2 = 8;
                    break;
                case "amp":
                    targetTID = 6;
                    break;
                case "source":
                    targetTID = 1;
                    targetTID2 = 2;
                    break;
                }
        }
        //determine if the primary tag id matches our target tag id
        sightedTID = LimelightHelpers.getFiducialID(limelightName);
        if (sightedTID == targetTID || sightedTID == targetTID2) {
            steerPID.enableContinuousInput(0.0, 360.0);
            tx = LimelightHelpers.getTX(limelightName);
            steeringAdjust = steerPID.calculate(tx);
            ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                stickX.getAsDouble()*speedLimit,
                stickY.getAsDouble()*speedLimit,
                steeringAdjust,
                Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset()));
            
            drivetrain.drive(ChassisSpeeds.discretize(fieldRelativeSpeeds, 0.020));
            SmartDashboard.putNumber("steringadjust",steeringAdjust);
            SmartDashboard.putNumber("tx",tx);
        } else { // just normal drive with no rotation
            SmartDashboard.putString("steeringadjust", "no valid TID");
            drivetrain.drive(ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                stickX.getAsDouble(),
                stickY.getAsDouble(),
                0,
                Rotation2d.fromDegrees(drivetrain.getGyroscopeAngle() + drivetrain.getGyroOffset())), 0.020));
        }
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("interupted");
        drivetrain.drive(new ChassisSpeeds(stickX.getAsDouble(), stickY.getAsDouble(), 0));
    }
}