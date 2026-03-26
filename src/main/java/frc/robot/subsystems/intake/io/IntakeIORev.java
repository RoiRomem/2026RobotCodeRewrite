package frc.robot.subsystems.intake.io;

import edu.wpi.first.math.MathUtil;

public class IntakeIORev extends IntakeIO {

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {

    }

    @Override
    public void setTargetAngle(double angle) {
        _closedLoopPivot = true;
        this._targetAngle = angle;
    }

    @Override
    public void runVoltsArm(double volts) {
        _closedLoopPivot = false;
    }

    @Override
    public void runVoltsRollers(double volts) {
        _rollerVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

}
