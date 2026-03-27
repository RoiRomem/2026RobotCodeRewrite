package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIORev;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import team6230.koiupstream.subsystems.ConditionalAction;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.sysid.AutoTuner;

public class Intake extends UpstreamSubsystem<RobotState, IntakeIO, IntakeIOInputsAutoLogged> {

    public AutoTuner tuner;

    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());

        addSuperstateBehaviour(RobotState.INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.IDLE, () -> {
            clearConditionalActions();
            io.setTargetAngle(IntakeConstants.kEncoderClosed);
        });
    }

    private void intaking() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kEncoderOpen);
        registerConditionalAction(new ConditionalAction(
                () -> io.getPivotAngleDeg() > IntakeConstants.kMinOpenAngle,
                () -> io.runVoltsRollers(IntakeConstants.kIntakingVolts)));
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isReady() {
        Logger.recordOutput("IntakeIsReady",
                Math.abs(io.getTargetAngle() - io.getPivotAngleDeg()) <= IntakeConstants.kErrorToleranceDeg);
        if (Superstate.getInstance().isCurrentWanted(RobotState.IDLE))
            return true;
        return Math.abs(io.getTargetAngle() - io.getPivotAngleDeg()) <= IntakeConstants.kErrorToleranceDeg;
    }

    @Override
    public IntakeIO getIO() {
        if (RobotBase.isSimulation()) {
            return new IntakeIOSim();
        }
        return new IntakeIORev();
    }

    public void setVoltage(double volts) {
        io.runVoltsPivot(volts);
    }

    public double getVelocity() {
        return io.getPivotVelocity();
    }

    public double getPosition() {
        return io.getPivotAngleDeg();
    }

    public void stopMotor() {
        io.stop();
    }
}