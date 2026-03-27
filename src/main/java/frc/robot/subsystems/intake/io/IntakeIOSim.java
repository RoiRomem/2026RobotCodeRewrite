package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOSim extends IntakeIO {
    private final DCMotorSim rollerSim;
    private final SingleJointedArmSim pivotSim;

    @AutoLogOutput
    public final LoggedMechanism2d _pivotMech;
    private final LoggedMechanismRoot2d _pivotRoot;
    private final LoggedMechanismLigament2d _pivotViz;

    private final ProfiledPIDController pivotController;
    private final ArmFeedforward pivotFeedforward;

    private double _targetAngleRad = 0.0;
    private boolean _closedLoopPivot = false;

    public IntakeIOSim() {

        var rollerMotor = DCMotor.getNEO(1).withReduction(IntakeConstants.kRollerGearRatio);
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(rollerMotor, 0.012, 0.0001), rollerMotor);

        pivotSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                IntakeConstants.kPivotGearRatio,
                IntakeConstants.kMOIpivot,
                IntakeConstants.kLengthPivot,
                IntakeConstants.kMinPhysicalAngleRad,
                IntakeConstants.kMaxPhysicalAngleRad,
                true,
                IntakeConstants.kMinPhysicalAngleRad,
                0.001, 0.001); // Encoder noise (angle, velo)

        _pivotMech = new LoggedMechanism2d(2.0, 2.0);
        _pivotRoot = _pivotMech.getRoot("Pivot", 1.0, 1.0);
        _pivotViz = _pivotRoot.append(
                new LoggedMechanismLigament2d("IntakePivot", IntakeConstants.kLengthPivot, 0));

        var constraints = new TrapezoidProfile.Constraints(
                IntakeConstants.kMaxVelocityRadPerSec,
                IntakeConstants.kMaxAccelRadPerSecSquared);

        pivotController = new ProfiledPIDController(
                IntakeConstants.kPsim,
                IntakeConstants.kIsim,
                IntakeConstants.kDsim,
                constraints);

        pivotFeedforward = new ArmFeedforward(
                IntakeConstants.kSsim,
                IntakeConstants.kGsim,
                IntakeConstants.kVsim,
                IntakeConstants.kAsim);

    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {

        if (DriverStation.isDisabled()) {
            runVoltsRollers(0);
            runVoltsPivot(0);
        }

        rollerSim.update(Constants.loopPeriodcSecs);

        if (_closedLoopPivot) {
            handlePivotClosedLoop();
        }

        pivotSim.update(Constants.loopPeriodcSecs);

        setInputs(inputs);
    }

    private void handlePivotClosedLoop() {
        double currentArmRad = pivotSim.getAngleRads();

        double pidOutput = pivotController.calculate(currentArmRad, _targetAngleRad);
        var setpoint = pivotController.getSetpoint();
        double ffOutput = pivotFeedforward.calculate(setpoint.position, setpoint.velocity);

        _pivotVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        pivotSim.setInputVoltage(_pivotVoltage);
    }

    @Override
    public void setTargetAngle(double targetEncoderDeg) {
        _closedLoopPivot = true;
        // 1. Convert the incoming Encoder Target to a Physical Arm Target
        double targetArmDeg = IntakeConstants.encoderToPivot(targetEncoderDeg);

        double clampedArmDeg = MathUtil.clamp(targetArmDeg, IntakeConstants.kPivotMinDeg, IntakeConstants.kPivotMaxDeg);

        this._targetAngle = clampedArmDeg;
        this._targetAngleRad = Math.toRadians(clampedArmDeg);
    }

    @Override
    public void runVoltsPivot(double volts) {
        _closedLoopPivot = false;
        _pivotVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        pivotSim.setInputVoltage(_pivotVoltage);
    }

    @Override
    public void runVoltsRollers(double volts) {
        _rollerVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        rollerSim.setInputVoltage(_rollerVoltage);
    }

    private void setInputs(IntakeIOInputsAutoLogged inputs) {
        double currentPivotDeg = Math.toDegrees(pivotSim.getAngleRads());

        double currentEncoderDeg = IntakeConstants.pivotToEncoder(currentPivotDeg);
        double targetEncoderDeg = IntakeConstants.pivotToEncoder(_targetAngle);

        inputs.pivotTargetAngle = targetEncoderDeg;
        inputs.pivotAngleError = Math.abs(targetEncoderDeg - currentEncoderDeg);
        inputs.absolutePivotAngleRad = Math.toRadians(getPivotAngleDeg());
        inputs.relativePivotAngleRad = Math.toRadians(getPivotAngleDeg());
        inputs.absolutePivotAngleDeg = currentEncoderDeg;
        inputs.relativePivotAngleDeg = getPivotAngleDeg();
        inputs.pivotClosedLoop = _closedLoopPivot;
        inputs.pivotAppliedVoltage = _pivotVoltage;
        inputs.pivotCurrent = new double[] { pivotSim.getCurrentDrawAmps() };
        inputs.rollerAppliedVoltage = _rollerVoltage;
        inputs.rollerCurrent = new double[] { rollerSim.getCurrentDrawAmps() };
        inputs.rollerRPM = rollerSim.getAngularVelocityRPM();

        _pivotViz.setAngle(currentPivotDeg + IntakeConstants.kEncoderOffsetSim);
    }

    @Override
    public double getPivotAngleDeg() {
        return IntakeConstants.pivotToEncoder(Math.toDegrees(pivotSim.getAngleRads()));
    }

    @Override
    public double getPivotVelocity() {
        return Math.toDegrees(pivotSim.getVelocityRadPerSec());
    }

    @Override
    public void stop() {
        pivotSim.setInputVoltage(0);
        rollerSim.setInput(0);
    }
}