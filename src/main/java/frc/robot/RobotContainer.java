package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import team6230.koiupstream.superstates.Superstate;

public class RobotContainer {

        private Superstate superstate = Superstate.getInstance();
        private CommandXboxController driverController = new CommandXboxController(0);

        // private final LoggedDashboardChooser<Command> autoChooser;

        private Trigger IntakeButton = driverController.leftTrigger();
        private Trigger HomeButton = driverController.a();
        private Trigger PreparingShooterButton = driverController.rightBumper();
        private Trigger ShootingButton = driverController.rightTrigger();
        private Trigger UnjamButton = driverController.povUp();

        private Intake intake = new Intake();
        private Drive drive = new Drive();

        public RobotContainer() {
                // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
                // AutoBuilder.buildAutoChooser());

                configureBindings();
        }

        private void configureBindings() {
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -driverController.getLeftY(),
                                                () -> -driverController.getLeftX(),
                                                () -> -driverController.getRightX()));
                superstate.setDefaultWantedState(RobotState.IDLE);

                IntakeButton
                                .and(PreparingShooterButton.negate()).and(ShootingButton.negate())
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.INTAKING));

                PreparingShooterButton
                                .and(ShootingButton.negate()).and(IntakeButton.negate())
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.PREPARING_SHOOTER));

                PreparingShooterButton
                                .and(ShootingButton.negate()).and(IntakeButton)
                                .whileTrue(superstate
                                                .setWantedSuperstateCommand(RobotState.PREPARING_SHOOTER_AND_INTAKING));

                ShootingButton
                                .and(IntakeButton.negate())
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.SHOOTING));

                ShootingButton
                                .and(IntakeButton)
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.SHOOTING_AND_INTAKING));

                UnjamButton
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.UNJAM));

                HomeButton
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.HOME));
        }

        public Command getAutonomousCommand() {
                // return autoChooser.get();
                return Commands.runOnce(
                                () -> drive.setPose(
                                                new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                                drive);
        }
}