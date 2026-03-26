package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIORev;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import team6230.koiupstream.subsystems.UpstreamSubsystem;

public class Intake extends UpstreamSubsystem<RobotState, IntakeIO, IntakeIOInputsAutoLogged> {
    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isReady() {
        return false;
    }

    @Override
    public IntakeIO getIO() {
        if (RobotBase.isSimulation()) {
            return new IntakeIOSim();
        }
        return new IntakeIORev();
    }
}