package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private SparkPIDController drivePidController;
    private SparkPIDController steerPidController;
    public double kP, kI, kD, kIz, drive_kFF, kMaxOutput, kMinOutput, steer_kFF;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANcoder absoluteSteerEncoder;

    public SwerveModule(int driveID, int steerID, int encoderID, double offset) {
        // Setup drive motor SparkMax
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(true);
        driveMotor.setSmartCurrentLimit(DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(DRIVE_SECONDARY_CURRENT_LIMIT);

        // Setup PID functionality for drive motors
        drivePidController = driveMotor.getPIDController();

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

        // Setup PID functionality for steer motors
        steerPidController = steerMotor.getPIDController();

        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(STEER_POSITION_CONVERSION);
        steerEncoder.setVelocityConversionFactor(STEER_VELOCITY_CONVERSION);

        // Setup steer motor absolute encoder
        absoluteSteerEncoder = new CANcoder(encoderID);

        // Zero encoders to ensure steer relative matches absolute
        resetEncoders();

        // PID coefficients
        kP = 6e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        
        // PID coefficients (drive)
        drive_kFF = 0.000015; 

        // PID coefficients (steer)
        steer_kFF = 0.0; 

        // set PID coefficients (drive)
        drivePidController.setP(kP);
        drivePidController.setI(kI);
        drivePidController.setD(kD);
        drivePidController.setIZone(kIz);
        drivePidController.setFF(drive_kFF);
        drivePidController.setOutputRange(kMinOutput, kMaxOutput);

        // set PID coefficients (steer)
        steerPidController.setP(STEER_P);
        steerPidController.setI(STEER_I);
        steerPidController.setD(STEER_D);
        steerPidController.setIZone(kIz);
        steerPidController.setFF(steer_kFF);
        steerPidController.setOutputRange(kMinOutput, kMaxOutput);
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
        drivePidController.setReference(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND, CANSparkMax.ControlType.kDutyCycle);
        steerPidController.setReference(state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        System.out.println("sms - " + state.toString());
    }

    /**
     * Locks the wheel at the provided angle
     * 
     * @param angle degrees
     */
    public void lockModule(int angle) {
        steerPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }

    public void updatePidValues() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("Drive P Gain", 0);
        double i = SmartDashboard.getNumber("Drive I Gain", 0);
        double d = SmartDashboard.getNumber("Drive D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double dff = SmartDashboard.getNumber("Drive Feed Forward", 0);
        double sff = SmartDashboard.getNumber("Steer Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { drivePidController.setP(p); kP = p; }
        if((i != kI)) { drivePidController.setI(i); kI = i; }
        if((d != kD)) { drivePidController.setD(d); kD = d; }
        if((iz != kIz)) { drivePidController.setIZone(iz); kIz = iz; }
        if((dff != drive_kFF)) { drivePidController.setFF(dff); drive_kFF = dff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            drivePidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max;
        }
        if((iz != kIz)) { steerPidController.setIZone(iz); kIz = iz; }
        if((sff != steer_kFF)) { steerPidController.setFF(sff); steer_kFF = sff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            steerPidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max;
        }
    }
}