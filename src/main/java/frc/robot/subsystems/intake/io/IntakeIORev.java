package frc.robot.subsystems.intake.io;

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
    public void setTargetRPM(double rpm) {
        _closedLoopRoller = true;
        this._targetRPM = rpm;
    }

    @Override
    public void runVoltsArm(double volts) {
        _closedLoopPivot = false;
    }

    @Override
    public void runVoltsRollers(double volts) {
        _closedLoopRoller = false;
    }

}
