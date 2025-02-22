// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

import frc.robot.sensors.*;

/*import frc.robot.interfaces.IElevator;
import frc.robot.interfaces.IDrawer;
import frc.robot.interfaces.INeck;
import frc.robot.interfaces.IRoller;*/

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Climber;
// import frc.robot.commands.climber.Elevator;
// import frc.robot.commands.climber.ElevatorUp;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.groups.*;
import frc.robot.commands.intake.CoralIntakeMovement;
//import frc.robot.commands.gamepad.*;
import frc.robot.auton.common.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;
	private boolean slowMode;
	private boolean fieldRelative = true;

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_STRAIGHT_FORWARD = "Move Straight Forward";
	public static final String AUTON_NEW_PATH = "New Path";
	private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();
	
	public static final String START_POSITION_1 = "Starting Position 1";
	public static final String START_POSITION_2 = "Starting Position 2";
	public static final String START_POSITION_3 = "Starting Position 3";
	public static final String START_POSITION_4 = "Starting Position 4";
	public static final String START_POSITION_5 = "Starting Position 5";
	public static final String START_POSITION_6 = "Starting Position 6";
	private String startPosition;
	private SendableChooser<String> startPositionChooser = new SendableChooser<>();

	public static final String MAIN_TARGET_CONE_NODE = "Cone Node";
	public static final String MAIN_TARGET_CUBE_NODE = "Cube Node";
	public static final String MAIN_TARGET_TWO_CONE_NODES = "Two Cone Nodes";
	public static final String MAIN_TARGET_TWO_CUBE_NODES = "Two Cube Nodes";
	public static final String MAIN_TARGET_CHARGING_STATION = "Charging Station";
	public static final String MAIN_TARGET_NOWHERE = "Nowhere";
	private String mainTarget;
	private SendableChooser<String> mainTargetChooser = new SendableChooser<>();

	// sensors

	private final HMAccelerometer accelerometer = new HMAccelerometer();

	// motorized devices

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
	public final Climber climber = new Climber();
	public final CoralIntake intake = new CoralIntake();
	// public final AddressableLED LED_Strip = new AddressableLED(Ports.PWM.LED_STRIP);
	public final AddressableLEDBuffer LED_StripBuffer = new AddressableLEDBuffer(8);
	public final CoralIntakeMovement intakeMove = new CoralIntakeMovement(intake, intake.m_LED, LED_StripBuffer);
	public double speedMult = 1;

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field

	// The driver's and copilot's joystick(s) and controller(s)

	CommandXboxController joyMain = new CommandXboxController(Ports.USB.MAIN_JOYSTICK);
	//CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		NamedCommands.registerCommand("lightOn", intakeMove);

		// choosers (for auton)
		autonChooser.setDefaultOption("Do Nothing", AUTON_DO_NOTHING);
		autonChooser.addOption("Move Straight Forward", AUTON_STRAIGHT_FORWARD);
		autonChooser.addOption("New Path", AUTON_NEW_PATH);
		SmartDashboard.putData("Auto choices", autonChooser);

		startPositionChooser.setDefaultOption("Starting Position 1", START_POSITION_1);
		startPositionChooser.addOption("Starting Position 2", START_POSITION_2);
		startPositionChooser.addOption("Starting Position 3", START_POSITION_3);
		startPositionChooser.addOption("Starting Position 4", START_POSITION_4);
		startPositionChooser.addOption("Starting Position 5", START_POSITION_5);
		startPositionChooser.addOption("Starting Position 6", START_POSITION_6);
		SmartDashboard.putData("Start positions", startPositionChooser);

		mainTargetChooser.setDefaultOption("To Nowhere", MAIN_TARGET_NOWHERE);
		mainTargetChooser.addOption("Cone Node", MAIN_TARGET_CONE_NODE);
		mainTargetChooser.addOption("Cube Node", MAIN_TARGET_CUBE_NODE);
		mainTargetChooser.addOption("Two Cone Nodes", MAIN_TARGET_TWO_CONE_NODES);
		mainTargetChooser.addOption("Two Cube Nodes", MAIN_TARGET_TWO_CUBE_NODES);
		mainTargetChooser.addOption("Charging Station", MAIN_TARGET_CHARGING_STATION);
		SmartDashboard.putData("Main targets", mainTargetChooser);


		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		// LED_Strip.setLength(LED_StripBuffer.getLength());
		// LED_Strip.setData(LED_StripBuffer);
		// LED_Strip.start();

		// intake.setDefaultCommand(new RunCommand(() -> intake.moveDowntoPos()));
		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(joyMain.getLeftY() * speedMult, JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getLeftX() * speedMult, JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getRightX() * speedMult, JOYSTICK_AXIS_THRESHOLD),
					fieldRelative, true),
				drivetrain));
	}	

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */

	public void toggleSpeed(){
		slowMode = !slowMode;

		if (slowMode == true) {
			speedMult = 0.25;
		} else {
			speedMult = 1;
		} 
	}

	public void toggleRelative(){
		fieldRelative = !fieldRelative;
		if (fieldRelative) {
			for(var i = 0; i < LED_StripBuffer.getLength(); i++) {
				LED_StripBuffer.setRGB(i, 255, 0, 100);
			}

			// LED_Strip.setData(LED_StripBuffer);
		} else {
			for(var i = 0; i < LED_StripBuffer.getLength(); i++) {
				LED_StripBuffer.setRGB(i, 0, 0, 0);
			}

			// LED_Strip.setData(LED_StripBuffer);
		}
	}

	private void configureButtonBindings() {

		/*------------------ JoyMain ------------------*/

		joyMain.button(5).onTrue(Commands.runOnce(() -> intakeMove.execute())); //command test :button:LB

		joyMain.button(6).onTrue(Commands.runOnce(() -> toggleSpeed())); //button:RB

		joyMain.start().onTrue(Commands.runOnce(() -> toggleRelative()));

		joyMain.button(1).whileTrue(Commands.runOnce(() -> intake.moveTest()));

		// joyMain.button(5).onTrue(Commands.runOnce(() -> climber.moveUp())); //button:LB

		// joyMain.povUp().onTrue(Commands.runOnce(() -> climber.moveUptoPos())); 

		// joyMain.povDown().onTrue(Commands.runOnce(() -> climber.moveDown()));

		// joyMain.button(1).onTrue(Commands.runOnce(() -> climber.resetClimbEncoder())); //button:a

		// joyMain.button(2).onTrue(Commands.runOnce(() -> climber.stopClimb())); //button:b

		joyMain.button(4).onTrue(new DrivetrainZeroHeading(drivetrain));	//button:y, resets the field to current robot direction for field-relative mode

		/*------------------ Copilot ------------------*/

		// copilotGamepad.povUp().onTrue(Commands.runOnce(() -> intake.moveUptoPos()));

		// copilotGamepad.povDown().onTrue(Commands.runOnce(() -> intake.moveDowntoPos()));

		// copilotGamepad.back().onTrue(Commands.runOnce(() -> intake.stopEverything()));

		// copilotGamepad.button(5).onTrue(Commands.runOnce(() -> intake.moveUp())); //button:LB

		// copilotGamepad.button(6).onTrue(Commands.runOnce(() -> intake.moveDown())); //button:RB

		// copilotGamepad.button(3).onTrue(Commands.runOnce(() -> intake.grabNote())); //button:x

		// copilotGamepad.button(2).onTrue(Commands.runOnce(() -> intake.dropNote())); //button:b

		// copilotGamepad.button(4).onTrue(Commands.runOnce(() -> intake.resetArmEncoder())); //button:y
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);		

		startPosition = startPositionChooser.getSelected();
		System.out.println("Start position: " + startPosition);

		mainTarget = mainTargetChooser.getSelected();
		System.out.println("Main target: " + mainTarget);
		
		// This switch connects the paths in pathplanner to the autonomous options

		switch (autonSelected) {

			case AUTON_NEW_PATH:
				return new PathPlannerAuto("New Path");	
				//break;

			case AUTON_STRAIGHT_FORWARD:
				return new PathPlannerAuto("straight");	
				//break;

			case AUTON_DO_NOTHING:
				return new PathPlannerAuto("Do Nothing");	
				//break;
				
			default:
				// nothing
				return null;
				//break;
		}
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	/*public Trajectory createExampleTrajectory() {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
			createTrajectoryConfig());

		return exampleTrajectory;
	}*/
	
	/*public Command createSwerveControllerCommand(Trajectory trajectory) {

		ProfiledPIDController thetaController = new ProfiledPIDController(
			AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory, // trajectory to follow
			drivetrain::getPose, // Functional interface to feed supplier
			DrivetrainConstants.DRIVE_KINEMATICS, // kinematics of the drivetrain
			new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for x position
			new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for y position
			thetaController, // trajectory tracker PID controller for rotation
			drivetrain::setModuleStates, // raw output module states from the position controllers
			drivetrain); // subsystems to require

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose()); // WARNING: https://github.com/REVrobotics/MAXSwerve-Java-Template/issues/13

		field.getObject("trajectory").setTrajectory(trajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
	}*/

	public Field2d getField()
	{
		return field;
	}

	public HMAccelerometer getAccelerometer()
	{
		return accelerometer;
	}

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public XboxController getMainJoystick()
	{
		return joyMain.getHID();
	}

	public XboxController getCopilotGamepad()
	{
		return copilotGamepad.getHID();
	}

	public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}

	public SendableChooser<String> getStartPositionChooser()
	{
		return startPositionChooser;
	}

	public SendableChooser<String> getMainTargetChooser()
	{
		return mainTargetChooser;
	}
}
