package frc.robot.util.roller.io;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.roller.RollerConfig;
import frc.robot.util.roller.RollerConfig.RollerMotor;

public class RollerIOSpark extends RollerIO {
    private SparkBase motor;

    private SparkClosedLoopController controller;

    public RollerIOSpark(RollerConfig config) {
        super(config);

        motor = config.motor == RollerMotor.NEO
                ? new SparkMax(config.motorId, MotorType.kBrushless)
                : new SparkFlex(config.motorId, MotorType.kBrushless);

        SparkBaseConfig m_config = config.motor == RollerMotor.NEO
                ? new SparkMaxConfig()
                : new SparkFlexConfig();

        m_config
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(IntakeConstants.kRollerCurrentLimits)
                .voltageCompensation(12);
        m_config.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        m_config.encoder.velocityConversionFactor(1 / IntakeConstants.kRollerGearRatio);

        motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        controller = motor.getClosedLoopController();
    }

    @Override
    public void runVoltage(double volt) {
        controller.setSetpoint(volt, ControlType.kVoltage);
    }

    @Override
    public double getRollerRadsPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    }

    @Override
    public void updateInputs(RollerIOInputsAutoLogged inputs) {
        inputs.appliedVoltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.radsPerSec = getRollerRadsPerSec();
        inputs.rollerCurrent = new double[] { motor.getOutputCurrent() };
    }
}
