package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import frc.robot.constants.Constants.PHYSICAL_CONSTANTS;
import frc.robot.constants.Constants.TUNED_CONSTANTS;
import frc.robot.utils.Conversions.WheelConversions;

public class SwerveModule {
    private CANcoder canCoder;

    private TalonFX driveMotor;
    private TalonFXConfiguration driveConfig;

    private SparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private SparkMaxConfig turnConfig;
    private EncoderConfig encoderConfig;

    private PIDController turnPID;

    /*
     * Uses for control driveMotor with .setControl() method
     */
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);   // OpenLoop
    private VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0); // CloseLoop
    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0, 0); // Feedforward for CloseLoop

    /**
     * @apiNote Constructor for SwerveModule used in {@link SwerveSubsystem}
     */
    public SwerveModule(int canCoderID, double CANCoderOffset, int driveID, boolean driveInverted, int turnID, boolean turnInverted){
        /* Configure CANCoder */
        canCoder = new CANcoder(canCoderID);

        canCoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withSensorDirection(
                    SensorDirectionValue.CounterClockwise_Positive
                ).withMagnetOffset(CANCoderOffset)
        );

        // BaseStatusSignal.setUpdateFrequencyForAll(100, canCoder.getAbsolutePosition());
        
        /* Configure Drive Motor */
        driveMotor = new TalonFX(driveID);

        /* Restore Factor Defualts */
        driveConfig = new TalonFXConfiguration();

        /* Current Limits */
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        driveConfig.CurrentLimits.StatorCurrentLimit = PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.DRIVE_CURRENT_LIMIT;

        /* Gear Ratio */
        driveConfig.Feedback.SensorToMechanismRatio = PHYSICAL_CONSTANTS.DRIVEBASE.GEARS.DRIVE_GEAR_RATIO;

        /* OpenLoop Control Ramp Period*/
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.DRIVE_LOOP_RAMP_RATE; // use with DutyCycleOut Control
        
        /* CloseLoop Control Ramp Period */
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0; // use with VelocityDutyCycle Control

        /* CloseLoop Constants, haven't tuned */
        driveConfig.Slot0 = new Slot0Configs()
            .withKP(TUNED_CONSTANTS.DRIVEBASE.DRIVE_PID_P)
            .withKI(TUNED_CONSTANTS.DRIVEBASE.DRIVE_PID_I);

        /* Motor Outputs */
        driveConfig.MotorOutput.Inverted = (driveInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Feedforward used for VelocityDutyCycle Output */
        driveFeedForward = new SimpleMotorFeedforward(
            TUNED_CONSTANTS.DRIVEBASE.DRIVE_FEED_FORWARD_KS,
            TUNED_CONSTANTS.DRIVEBASE.DRIVE_FEED_FORWARD_KV,
            TUNED_CONSTANTS.DRIVEBASE.DRIVE_FEED_FORWARD_KA
            // No need for kG because it's not arm/elevator
        );

        // BaseStatusSignal.setUpdateFrequencyForAll(60, driveMotor.getVelocity(), driveMotor.getPosition());

        /* Apply Settings */
        driveMotor.getConfigurator().apply(driveConfig);

        /* Configure Turn Motor */
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();

        /* Create Configuration Objects */
        turnConfig = new SparkMaxConfig();
        encoderConfig = new EncoderConfig();

        /* Current Limits */
        turnConfig.voltageCompensation(PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);
        turnConfig.smartCurrentLimit(PHYSICAL_CONSTANTS.DRIVEBASE.LIMITING.TURN_CURRENT_LIMIT);
        
        /* Gear Ratio */
        encoderConfig.positionConversionFactor((1.0 / PHYSICAL_CONSTANTS.DRIVEBASE.GEARS.TURN_GEAR_RATIO) * Math.PI * 2);
        encoderConfig.velocityConversionFactor((1.0 / PHYSICAL_CONSTANTS.DRIVEBASE.GEARS.TURN_GEAR_RATIO) * Math.PI * 2);

        /* PID Constructor */
        turnPID = new PIDController(
            TUNED_CONSTANTS.DRIVEBASE.TURN_PID_P,
            TUNED_CONSTANTS.DRIVEBASE.TURN_PID_I,
            TUNED_CONSTANTS.DRIVEBASE.TURN_PID_D    
        );

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        /* Motor Outputs */
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.inverted(turnInverted);

        /* Apply Settings */
        turnConfig.apply(encoderConfig);
        turnMotor.configure(
            turnConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * get the absolute angle of CANCoder, with an delay of 200ms
     * @return the angle in Radians
     */
    private double getAbsoluteAngleRad() {
        return canCoder.getAbsolutePosition().waitForUpdate(0.25).getValueAsDouble() * Math.PI * 2; 
    }

    /**
     * get the velocity of Drive Motor
     * @return velocity in meters per second
     */
    private double getDriveVelocity() {
        return WheelConversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
    }

    /**
     * get the position of Drive Motor
     * @return distance in meters
     */
    private double getDriveDistance() {
        return WheelConversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(),  PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
    }

    /**
     * get the postion returns from turn
     * @return angle in radians
     */
    private double getTurnAngleRad(){
        return turnEncoder.getPosition();
    }

    /**
     * get the angle returns from turn's encoder
     * @return angle in {@link Rotation2d}
     */
    private Rotation2d getTurnRotation2D() {
        return Rotation2d.fromRadians(getTurnAngleRad());
    }

    /**
     * get the position(distane and angle) of Swerve Module
     * @return position in {@link SwerveModulePosition} 
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            getTurnRotation2D()
        );
    }

    /**
     * get the state(velocity and angle) of Swerve Module
     * @return state in {@link SwerveModulePosition} 
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            getTurnRotation2D()
        );
    }

    /**
     * set the speed for perpotion
     * @param speed the optimized {@link SwerveModuleState}'s speed
     * @param isDutyCycle control mode, true for {@link DutyCycleOut} Control, false for {@link VelocityDutyCycle}
     */
    public void setDrive(double speed, boolean isDutyCycle) {
        if(isDutyCycle){
            dutyCycle.Output = speed / PHYSICAL_CONSTANTS.DRIVEBASE.MAX_SPEED_METERS;
            driveMotor.setControl(dutyCycle);
        }else{
            Measure<? extends PerUnit<DistanceUnit, TimeUnit>> currentVelocity = Units.MetersPerSecond.of(getDriveVelocity());
            Measure<? extends PerUnit<DistanceUnit, TimeUnit>> nextVelocity = Units.MetersPerSecond.of(speed);

            velocityDutyCycle.Velocity = WheelConversions.MPSToRPS(speed, PHYSICAL_CONSTANTS.DRIVEBASE.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
            velocityDutyCycle.FeedForward = driveFeedForward.calculate(currentVelocity, nextVelocity).magnitude();

            driveMotor.setControl(velocityDutyCycle);
        }
    }

    /**
     * Uses for SysID tuning
     * @param voltage voltage output
     */
    public void setDriveVoltage(double voltage){
        driveMotor.setControl(new VoltageOut(voltage));
    }

    /**
     * set the angle for turn
     * @param desireAngle the optimized {@link SwerveModuleState}'s angle in radians
     */
    public void setTurn(double desireAngle){
        turnMotor.set(turnPID.calculate(getTurnAngleRad(), desireAngle));
    }

    /**
     * set the state of {@link SwerveModule}
     * @param desireState the desired {@link SwerveModuleState} 
     * @param isDutyCycle control mode, duty cycle is true, velocity is false
     */
    public void setState(SwerveModuleState desireState, boolean isDutyCycle){
        /* Stop Module if speed < 1% */
        if(Math.abs(desireState.speedMetersPerSecond) < 0.05){
            stopModule();
            return;
        }

        /*Optimization and Cosine compensation for drive */
        Rotation2d currnetAngle = getModuleState().angle;
        desireState.optimize(currnetAngle);
        desireState.speedMetersPerSecond *= desireState.angle.minus(currnetAngle).getCos();

        setDrive(desireState.speedMetersPerSecond, isDutyCycle);
        setTurn(desireState.angle.getRadians());
    }

    /**
     * stop motors
     */
    public void stopModule(){
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    /**
     * reset position of encoders to default value
     * @apiNote drive to 0, turn to CANCoder's position
     */
    public void resetEncoders(){
        driveMotor.getConfigurator().setPosition(0.0); // not used in openloop
        turnEncoder.setPosition(getAbsoluteAngleRad());
    }
}
