package frc.robot.subsystems.intake.io;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIORev extends IntakeIO {
    private SparkMax m_pivot, m_roller;
    private RelativeEncoder m_relativeEncoder, m_rollerEncoder;
    private DutyCycleEncoder m_absoluteEncoder;
    private SparkClosedLoopController m_pivotController, m_rollerController;

    public IntakeIORev() {
        m_pivot = new SparkMax(RobotMap.CanBus.kIntakePivotID, MotorType.kBrushless);
        m_roller = new SparkMax(RobotMap.CanBus.kIntakeRollerID, MotorType.kBrushless);

        m_relativeEncoder = m_pivot.getEncoder();
        m_rollerEncoder = m_roller.getEncoder();

        m_absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.kIntakePivotThroughBoreID,
                IntakeConstants.kThroughBoreRange,
                IntakeConstants.kThroughBoreOffset);

        var pivotConfig = new SparkMaxConfig();
        pivotConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12);
        pivotConfig
                .smartCurrentLimit(IntakeConstants.kPivotCurrentLimits).softLimit
                .forwardSoftLimit(IntakeConstants.kForwardSoftLimit)
                .reverseSoftLimit(IntakeConstants.kReverseSoftLimit);

        pivotConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        pivotConfig.closedLoop
                .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD).feedForward
                .kS(IntakeConstants.kS)
                .kV(IntakeConstants.kV)
                .kA(IntakeConstants.kA)
                .kCos(IntakeConstants.kG)
                .kCosRatio(IntakeConstants.kCosRatio);

        pivotConfig.closedLoop.maxMotion
                .maxAcceleration(IntakeConstants.kMaxAcceleration)
                .allowedProfileError(IntakeConstants.kTolerance)
                .cruiseVelocity(IntakeConstants.kCruiseVelocity)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        pivotConfig.encoder.positionConversionFactor(360.0 / IntakeConstants.kPivotGearRatio)
                .velocityConversionFactor((360.0 / IntakeConstants.kPivotGearRatio) / 60.0);

        m_pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_pivotController = m_pivot.getClosedLoopController();

        var rollerConfig = new SparkMaxConfig();

        rollerConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(IntakeConstants.kRollerCurrentLimits)
                .voltageCompensation(12);
        rollerConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        rollerConfig.encoder.velocityConversionFactor(1 / IntakeConstants.kRollerGearRatio);

        m_roller.configure(rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_rollerController = m_roller.getClosedLoopController();

        m_relativeEncoder.setPosition(m_absoluteEncoder.get());
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.absolutePivotAngleDeg = m_absoluteEncoder.get();
        inputs.absolutePivotAngleRad = Math.toRadians(m_absoluteEncoder.get());
        inputs.pivotAngleError = Math.abs(_targetAngle - m_relativeEncoder.getPosition());
        inputs.pivotAppliedVoltage = m_pivot.getAppliedOutput() * m_pivot.getBusVoltage();
        inputs.pivotClosedLoop = _closedLoopPivot;
        inputs.pivotCurrent = new double[] { m_pivot.getOutputCurrent() };
        inputs.pivotTargetAngle = _targetAngle;
        inputs.relativePivotAngleDeg = m_relativeEncoder.getPosition();
        inputs.relativePivotAngleRad = Math.toRadians(m_relativeEncoder.getPosition());
        inputs.rollerAppliedVoltage = m_roller.getAppliedOutput() * m_roller.getBusVoltage();
        inputs.rollerCurrent = new double[] { m_roller.getOutputCurrent() };
        inputs.rollerRPM = m_rollerEncoder.getVelocity();
    }

    @Override
    public void setTargetAngle(double angle) {
        _closedLoopPivot = true;
        this._targetAngle = angle;
        m_pivotController.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void runVoltsPivot(double volts) {
        _closedLoopPivot = false;
        m_pivotController.setSetpoint(volts, ControlType.kVoltage);
    }

    @Override
    public void runVoltsRollers(double volts) {
        _rollerVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        m_rollerController.setSetpoint(volts, ControlType.kVoltage);
    }

    @Override
    public double getPivotAngleDeg() {
        return m_relativeEncoder.getPosition();
    }

    @Override
    public double getPivotVelocity() {
        return m_relativeEncoder.getVelocity();
    }

    @Override
    public void stop() {
        m_pivot.stopMotor();
        m_roller.stopMotor();
    }

}
