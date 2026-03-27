package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIORev;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import team6230.koiupstream.subsystems.ConditionalAction;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.tunable.Tunable;
import team6230.koiupstream.tunable.TunableManager;

public class Intake extends UpstreamSubsystem<RobotState, IntakeIO, IntakeIOInputsAutoLogged> {

    @Tunable
    public double kP = IntakeConstants.kPsim;
    @Tunable
    public double kI = IntakeConstants.kIsim;
    @Tunable
    public double kD = IntakeConstants.kDsim;
    @Tunable
    public double kS = IntakeConstants.kSsim;
    @Tunable
    public double kV = IntakeConstants.kVsim;
    @Tunable
    public double kG = IntakeConstants.kGsim;
    @Tunable
    public double kA = IntakeConstants.kAsim;

    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());

        addSuperstateBehaviour(RobotState.INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.IDLE, () -> {
            clearConditionalActions();
            io.setTargetAngle(IntakeConstants.kEncoderClosed);
        });
        addSuperstateBehaviour(RobotState.HOME, () -> home());

        // setSuperstateMode(false);
    }

    public void home() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kEncoderClosed);
        io.runVoltsRollers(0);
    }

    private void intaking() {
        clearConditionalActions();
        io.setTargetAngle(150);
        registerConditionalAction(new ConditionalAction(
                () -> io.getPivotAngleDeg() > IntakeConstants.kMinOpenAngle,
                () -> io.runVoltsRollers(IntakeConstants.kIntakingVolts)));
    }

    @Override
    public void update() {
        if (DriverStation.isTest()) {
            setSuperstateMode(false);
            if (TunableManager.checkChanged(this)) {
                io.setPIDF(kP, kI, kD, kS, kG, kV, kA);
                System.out.println("changed");
            }
        } else {
            setSuperstateMode(true);
        }
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

    public Command joystickControlCommand(DoubleSupplier axis) {
        return run(() -> {
            io.runVoltsPivot(MathUtil.clamp(axis.getAsDouble() * 12.0, -12, 12));
        });
    }
}