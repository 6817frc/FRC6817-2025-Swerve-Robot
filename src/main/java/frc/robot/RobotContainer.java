// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

import frc.robot.sensors.*;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Climber;
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
	public double speedvalue = 0.1;

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
				() -> updateJoystick(
					-MathUtil.applyDeadband(joyMain.getLeftY() * speedMult, JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getLeftX() * speedMult, JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getRightX() * speedMult, JOYSTICK_AXIS_THRESHOLD),
					fieldRelative, true, 
					MathUtil.applyDeadband(copilotGamepad.getLeftY(), JOYSTICK_AXIS_THRESHOLD), 
					MathUtil.applyDeadband(copilotGamepad.getRightY(), JOYSTICK_AXIS_THRESHOLD),
					MathUtil.applyDeadband(joyMain.getLeftTriggerAxis(), JOYSTICK_AXIS_THRESHOLD),
					MathUtil.applyDeadband(joyMain.getRightTriggerAxis(), JOYSTICK_AXIS_THRESHOLD)),
				drivetrain, intake));
				
		
		// Basic targeting data
		double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
		double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
		double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
		boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

		double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
		double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees

	
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
private boolean cameraTargetingMode;

double kMaxSpeed = 3.0;

 // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .1;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to speed for our drive method
    targetingAngularVelocity *= kMaxSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

	public void updateJoystick(double xSpeed, double ySpeed, double rot, Boolean fieldRelative, 
	Boolean rateLimit, double leftYValue, double RightYValue, double leftTrig, double rightTrig) {

// while the B-button is pressed, overwrite some of the driving values with the output of our limelight methods
		if (cameraTargetingMode == true) {
			final var side_limelight = limelight_aim_proportional();
      		ySpeed = side_limelight;

        	final var forward_limelight = limelight_range_proportional();
        	xSpeed = forward_limelight;

        	//while using Limelight, turn off field-relative driving.
        	fieldRelative = false;
		}
		drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
		intake.armMove(leftYValue);
		intake.wristMove(RightYValue);
		intake.wheelOut(leftTrig);
		intake.wheelIn(rightTrig);
	}

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

	public void speedIncrement() {
		if (speedvalue < 1.0) {
			speedvalue += 0.025;
		} else {
			speedvalue = 0.1;
		}
	}

	public void buttonPressed() {
		cameraTargetingMode = true;
	}

	public void buttonNotPressed() {
		cameraTargetingMode = false;
	}

	private void configureButtonBindings() {

		/*------------------ JoyMain ------------------*/

		joyMain.button(2).onTrue(Commands.runOnce(() -> buttonPressed()));

		joyMain.button(2).onFalse(Commands.runOnce(() -> buttonNotPressed()));

		joyMain.start().onTrue(Commands.runOnce(() -> toggleRelative()));

		joyMain.button(1).onTrue(Commands.runOnce(() -> intake.wheelOutTest(speedvalue))); //button:a

		joyMain.button(1).onFalse(Commands.runOnce(() -> intake.stopWheels())); //button:a
		
		joyMain.button(3).onTrue(Commands.runOnce(() -> speedIncrement()));

		joyMain.button(5).onTrue(Commands.runOnce(() -> intake.wheelOutFast())); //button:LB

		joyMain.button(5).onFalse(Commands.runOnce(() -> intake.stopWheels())); //button:LB

		joyMain.button(4).onTrue(new DrivetrainZeroHeading(drivetrain));	//button:y, resets the field to current robot direction for field-relative mode

		/*------------------ Copilot ------------------*/

		copilotGamepad.povDown().onTrue(Commands.runOnce(() -> intake.armL1()));

		copilotGamepad.povLeft().onTrue(Commands.runOnce(() -> intake.armL2()));

		copilotGamepad.povUp().onTrue(Commands.runOnce(() -> intake.armL3()));

		copilotGamepad.povRight().onTrue(Commands.runOnce(() -> intake.armL4()));

		copilotGamepad.button(4).onTrue(Commands.runOnce(() -> climber.climbMove()));

		copilotGamepad.button(4).onFalse(Commands.runOnce(() -> climber.stopClimb()));

		copilotGamepad.button(3).onTrue(Commands.runOnce(() -> intake.armIntake()));

		copilotGamepad.button(2).onTrue(Commands.runOnce(() -> climber.climbMoveRev()));

		copilotGamepad.button(2).onFalse(Commands.runOnce(() -> climber.stopClimb()));

		copilotGamepad.button(1).onTrue(Commands.runOnce(() -> intake.armSafe()));
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
