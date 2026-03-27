package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake;
import team6230.koiupstream.superstates.Superstate;

public class RobotContainer {

  private Superstate superstate = Superstate.getInstance();
  private CommandXboxController driverController = new CommandXboxController(0);

  private Trigger IntakeButton = driverController.leftBumper();

  private Intake intake = new Intake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    superstate.setDefaultWantedState(RobotState.IDLE);

    IntakeButton.whileTrue(superstate.setWantedSuperstateCommand(RobotState.INTAKING));

    /*
     * driverController.a().whileTrue(intake.tuner.quasistaticForward());
     * driverController.b().whileTrue(intake.tuner.quasistaticBackward());
     * driverController.y().whileTrue(intake.tuner.dynamicForward());
     * driverController.x().whileTrue(intake.tuner.dynamicBackward());
     * 
     * driverController.povUp().whileTrue(intake.tuner.analyze());
     */
    driverController.a().whileTrue(new SequentialCommandGroup(intake.tuner.quasistaticForward(),
        intake.tuner.quasistaticBackward(), intake.tuner.dynamicForward(), intake.tuner.analyze()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
