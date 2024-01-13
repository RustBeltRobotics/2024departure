package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANcoder absoluteSteerEncoder;

    private final PIDController steerPID = new PIDController(STEER_P, STEER_I, STEER_D);

    public SwerveModule(int driveID, int steerID, int encoderID, double offset) {
        // Setup drive motor SparkMax
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(true);
        driveMotor.setSmartCurrentLimit(DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(DRIVE_SECONDARY_CURRENT_LIMIT);

        // Setup drive motor relative encoder
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);

        // Setup steer motor SparkMax
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(true);
        steerMotor.setSmartCurrentLimit(STEER_SMART_CURRENT_LIMIT);
        steerMotor.setSecondaryCurrentLimit(STEER_SECONDARY_CURRENT_LIMIT);

        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(STEER_POSITION_CONVERSION);
        steerEncoder.setVelocityConversionFactor(STEER_VELOCITY_CONVERSION);

        // Setup steer motor absolute encoder
        absoluteSteerEncoder = new CANcoder(encoderID);

        // Allow PID to loop over
        steerPID.enableContinuousInput(0., 360.);

        // Zero encoders to ensure steer relative matches absolute
        resetEncoders();
    }

    /** @return Drive position, meters, -inf to +inf */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Steer position, degrees, -inf to +inf */
    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    /** @return Drive position, meters/second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /** @return Steer position, degrees/second */
    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    /** @return Absolute steer position, degrees, -inf to +inf */
    public double getAbsolutePosition() {
        return absoluteSteerEncoder.getPosition().getValueAsDouble() * 360.;
    }

    /** @return Drive encoder (meters) and steer encoder (Rotation2d) positions */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Resets the drive relative encoder to 0 and steer relative encoder to match
     * absolute encoder
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0.);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Set's the speed and angle of an idividual module.
     * 
     * @param state the desired state (velocity, m/s, and steer angle, Rotation2d)
     */
    public void setState(SwerveModuleState state) {
        // If input is minimal, ignore input to avoid reseting steer angle to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stopModule();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND);
        steerMotor.set(steerPID.calculate(getSteerPosition(), state.angle.getDegrees()));
    }

    /**
     * Locks the wheel at the provided angle
     * 
     * @param angle degrees
     */
    public void lockModule(int angle) {
        steerMotor.set(steerPID.calculate(getSteerPosition(), angle));
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }
}
