package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.NEO_SECONDARY_CURRENT_LIMIT;
import static frc.robot.Constants.NEO_SMART_CURRENT_LIMIT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private final Encoder arm1AbsEncoder;
    private final SparkPIDController arm1PidController;
    private final SparkPIDController arm2PidController;
    public double kP, kI, kD, kIz, kMaxOutput, kMinOutput, kFF;

    public Arm(){
        //set motor things
        armMotor1 = new CANSparkMax(97, MotorType.kBrushless);
        armMotor1.restoreFactoryDefaults();
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(false);
        armMotor1.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor1.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        armMotor2 = new CANSparkMax(97, MotorType.kBrushless);
        armMotor2.restoreFactoryDefaults();
        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor2.setInverted(false);
        armMotor2.setSmartCurrentLimit(NEO_SMART_CURRENT_LIMIT);
        armMotor2.setSecondaryCurrentLimit(NEO_SECONDARY_CURRENT_LIMIT);

        arm1AbsEncoder = new Encoder(0,1);

        // PID coefficients
        kP = 7e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1; 
        kMinOutput = -1;

        //set pid things
        arm1PidController = armMotor1.getPIDController();
        arm1PidController.setP(kP);
        arm1PidController.setI(kI);
        arm1PidController.setD(kD);
        arm1PidController.setIZone(kIz);
        arm1PidController.setFF(kFF);
        arm1PidController.setOutputRange(kMinOutput, kMaxOutput);
        arm1PidController.setPositionPIDWrappingEnabled(true);

        arm2PidController = armMotor2.getPIDController();
        arm2PidController.setP(kP);
        arm2PidController.setI(kI);
        arm2PidController.setD(kD);
        arm2PidController.setIZone(kIz);
        arm2PidController.setFF(kFF);
        arm2PidController.setOutputRange(kMinOutput, kMaxOutput);
        arm2PidController.setPositionPIDWrappingEnabled(true);
    }
    public void spinShooter(double velocity){
        arm1PidController.setReference(velocity, ControlType.kVelocity);
        arm2PidController.setReference(velocity, ControlType.kVelocity);
    }
    public void stopShooter(){
        arm1PidController.setReference(0, ControlType.kVelocity);
        arm2PidController.setReference(0, ControlType.kVelocity);
    }
}
