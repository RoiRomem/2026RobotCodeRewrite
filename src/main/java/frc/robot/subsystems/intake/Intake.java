package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIORev;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import team6230.koiupstream.subsystems.ConditionalAction;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;

public class Intake extends UpstreamSubsystem<RobotState, IntakeIO, IntakeIOInputsAutoLogged> {
    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());

        addSuperstateBehaviour(RobotState.INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.IDLE, () -> {
            clearConditionalActions();
            io.setTargetAngle(IntakeConstants.kClosedAngle);
        });
    }

    private void intaking() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kOpenAngle);
        registerConditionalAction(new ConditionalAction(
                () -> io.getPivotAngleDeg() > IntakeConstants.kMinOpenAngle,
                () -> io.runVoltsRollers(IntakeConstants.kIntakingVolts)));
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isReady() {
        if (Superstate.getInstance().isCurrent(RobotState.IDLE))
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
}