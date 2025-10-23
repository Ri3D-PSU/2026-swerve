package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;


public class RobotContainer {

    public static final double MAX_LINEAR_SPEED = 0.5;//Units.feetToMeters(17.2);
    public static final double MAX_ANGULAR_SPEED = Math.PI;
    private final Drive drive = new Drive();


    private final CommandXboxController m_driverController = new CommandXboxController(0);


    public RobotContainer() {

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(
                new RunCommand(() -> drive.drive(
                        Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY() * MAX_LINEAR_SPEED * -1,
                        Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX() * MAX_LINEAR_SPEED * -1,
                        Math.abs(m_driverController.getRightX()) * m_driverController.getRightX() * MAX_ANGULAR_SPEED * -1,
                        true), drive));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
