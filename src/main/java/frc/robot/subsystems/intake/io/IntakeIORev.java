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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIORev extends IntakeIO {
    private SparkMax m_pivot;
    private RelativeEncoder m_relativeEncoder;
    private DutyCycleEncoder m_absoluteEncoder;
    private SparkClosedLoopController m_pivotController;

    public IntakeIORev() {
        m_pivot = new SparkMax(RobotMap.CanBus.kIntakePivotID, MotorType.kBrushless);

        m_relativeEncoder = m_pivot.getEncoder();

        m_absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.kIntakePivotThroughBoreID,
                360 / IntakeConstants.kPivotShaftToPivotGearRatio,
                IntakeConstants.kThroughBoreOffset);

        var pivotConfig = new SparkMaxConfig();
        pivotConfig
                .inverted(IntakeConstants.kMotorInverted)
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

        pivotConfig.encoder
                .inverted(IntakeConstants.kEncoderInverted)
                .positionConversionFactor(360.0 / IntakeConstants.kPivotMotorToPivotGearRatio)
                .velocityConversionFactor((360.0 / IntakeConstants.kPivotMotorToPivotGearRatio) / 60.0);

        m_pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        m_pivotController = m_pivot.getClosedLoopController();

        m_relativeEncoder.setPosition(getAbsoluteEncoderDeg());
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.absolutePivotAngleDeg = getAbsoluteEncoderDeg();
        inputs.absolutePivotAngleRad = Math.toRadians(getAbsoluteEncoderDeg());
        inputs.pivotAngleError = Math.abs(_targetAngle.getDegrees() - m_relativeEncoder.getPosition());
        inputs.pivotAppliedVoltage = m_pivot.getAppliedOutput() * m_pivot.getBusVoltage();
        inputs.pivotClosedLoop = _closedLoopPivot;
        inputs.pivotCurrent = new double[] { m_pivot.getOutputCurrent() };
        inputs.pivotTargetAngle = _targetAngle.getDegrees();
        inputs.relativePivotAngleDeg = m_relativeEncoder.getPosition();
        inputs.relativePivotAngleRad = Math.toRadians(m_relativeEncoder.getPosition());
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        _closedLoopPivot = true;
        this._targetAngle = angle;
        m_pivotController.setSetpoint(angle.getDegrees(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void runVoltsPivot(double volts) {
        _closedLoopPivot = false;
        m_pivotController.setSetpoint(volts, ControlType.kVoltage);
    }

    @Override
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromDegrees(m_relativeEncoder.getPosition());
    }

    @Override
    public Rotation2d getPivotVelocity() {
        return Rotation2d.fromDegrees(m_relativeEncoder.getVelocity());
    }

    @Override
    public void stop() {
        m_pivot.stopMotor();
    }

    @Override
    public void setPIDF(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        var config = new SparkMaxConfig();
        config.closedLoop
                .pid(kP, kI, kD).feedForward
                .kS(kS)
                .kCos(kG)
                .kV(kV)
                .kA(kA);

        m_pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private double getAbsoluteEncoderDeg() {
        var raw = m_absoluteEncoder.get();

        return IntakeConstants.kEncoderInverted ? (360 / IntakeConstants.kPivotShaftToPivotGearRatio - raw) : raw;
    }
}
