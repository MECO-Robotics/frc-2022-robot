// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

/**
 * A tank drive train driven by dual motors for each side.
 */
public class DriveSubsystem extends SubsystemBase {

  // The type of encoder indicates the resolution - such as the Grayhill 63R128
  private static final int ENCODER_RESOLUTION = 128;
  private static final double WHEEL_DIAMETER_INCHES = 6.0; // inches
  private static final double WHEEL_CIRCUM_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
  public static final double TRACK_WIDTH_METERS = 0.71; // 28" Wheelbase in meters. Distance from center of left wheels
                                                        // to center of right wheels

  // This object defines the properties of how the robot turns
  public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);

  // Real sensors & actuators
  private MotorControllerGroup leftMotors, rightMotors;
  private Encoder leftEncoder, rightEncoder;
  private RangeSensor rangeLeft = new RangeSensor(Constants.LEFT_ULTRASONIC_ANLG);
  private RangeSensor rangeMiddle = new RangeSensor(Constants.MID_ULTRASONIC_ANLG);
  private RangeSensor rangeRight = new RangeSensor(Constants.RIGHT_ULTRASONIC_ANLG);

  // The Inertial Measurement Unit (IMU) connects to the RoboRio through the MXP
  // port,
  // which is the wide female pin header in the middle of the Rio. Through the
  // MXP,
  // the Serial Port Interface (SPI) is used.
  private final AHRS imu = new AHRS(SPI.Port.kMXP);

  // Converts tank or arcade drive speed requests to voltage requests to the motor
  // controllers
  private final DifferentialDrive drive;

  // Keeps track of where we are on the field based on gyro and encoder inputs.
  private final DifferentialDriveOdometry odometry;

  // Allows changing the the max demand change from the default defined in
  // Constants
  private final NetworkTableEntry speedRamp, turnRamp;

  // This member will limit acceleration to reduce skid
  // Limit is units of max drive input change per second. Drive input is 0 to 1
  // for stop to full speed,
  // so a value of 1 would say it would take one second to from stop to full
  // speed.
  // A value of 5 would say it takes 200ms to reach full speed.
  SlewRateLimiter arcadeThrottleRamp = new SlewRateLimiter(1f / Constants.DEFAULT_MAX_DEMAND_CHANGE);
  SlewRateLimiter arcadeTurnRamp = new SlewRateLimiter(1f / Constants.DEFAULT_MAX_DEMAND_CHANGE);

  // Sensor simulations
  private final Field2d field2d = new Field2d();
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  private final SimDeviceSim imuSim;

  // Drivetrain sim using standard kit of parts
  private final DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      KitbotGearing.k10p71, // 10.71:1
      KitbotWheelSize.kSixInch, // 6" diameter wheels.
      null); // No measurement noise.

  // Simple "fake" 2 gear system. For this drive train, it just changes the speed
  // range.
  private int gear = 1;
  private double GEAR_REDUCTION[] = new double[] { 0.66, 1.0 };
  private double reduction = 0.66F;

  /** Creates a new Subsystem. */
  public DriveSubsystem() {

    // Create the WPI_VictorSPX individually and call configOpenLoopRamp on it.
    WPI_VictorSPX left1 = new WPI_VictorSPX(Constants.RIGHT_DRIVE_1_CAN);
    WPI_VictorSPX left2 = new WPI_VictorSPX(Constants.RIGHT_DRIVE_2_CAN);
    WPI_VictorSPX left3 = new WPI_VictorSPX(Constants.RIGHT_DRIVE_3_CAN);
    WPI_VictorSPX right1 = new WPI_VictorSPX(Constants.LEFT_DRIVE_1_CAN);
    WPI_VictorSPX right2 = new WPI_VictorSPX(Constants.LEFT_DRIVE_2_CAN);
    WPI_VictorSPX right3 = new WPI_VictorSPX(Constants.LEFT_DRIVE_3_CAN);

    left1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    left3.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
    right3.setNeutralMode(NeutralMode.Brake);

    leftMotors = new MotorControllerGroup(left1, left2, left3);
    rightMotors = new MotorControllerGroup(right1, right2, right3);

    // So positive values cause forward movement
    leftMotors.setInverted(false);
    rightMotors.setInverted(true);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    // Left: reverse direction (decreasing values go forward)
    leftEncoder = new Encoder(Constants.LEFT_DRIVE_ENCODER_1_DIO, Constants.LEFT_DRIVE_ENCODER_2_DIO, true);
    // Right
    rightEncoder = new Encoder(Constants.RIGHT_DRIVE_ENCODER_1_DIO, Constants.RIGHT_DRIVE_ENCODER_2_DIO, false);

    leftEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(WHEEL_CIRCUM_METERS / ENCODER_RESOLUTION);

    if (RobotBase.isSimulation()) {
      // At some point, we'll need to reconfigure the drive sim from the defaults
      // since we're using a custom drive train
      // driveSim = new DifferentialDrivetrainSim(DCMotor.getCIM(3), 10f,
      // jKgMetersSquared, massKg, wheelRadiusMeters, trackWidthMeters,
      // measurementStdDevs)

      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
      imuSim = new SimDeviceSim("navX-Sensor[0]");

    } else {
      leftEncoderSim = null;
      rightEncoderSim = null;
      imuSim = null;
    }

    // Set the initial position (0,0) and heading (whatever it is) of the robot on
    // the field
    odometry = new DifferentialDriveOdometry(imu.getRotation2d());

    addChild("Left Motors", leftMotors);
    addChild("Left Encoder", leftEncoder);
    addChild("Right Encoder", rightEncoder);
    addChild("Right Motors", rightMotors);
    addChild("Diff Drive", drive);
    addChild("IMU heading", imu);// :)
    addChild("Field", field2d);
    addChild("Left Range", rangeLeft);
    addChild("Middle Range", rangeMiddle);
    addChild("Right Range", rangeRight);

    speedRamp = Shuffleboard.getTab("Drive")
        .add("Speed Ramp (time to go 0-Full)", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.010, "max", 2.000)) // Can't allow zero or it will crash
        .getEntry();

    speedRamp.setDouble(Constants.DEFAULT_MAX_DEMAND_CHANGE);
    speedRamp.addListener(
        (entryNotification) -> {
          System.out.println("Speed Ramp changed value: " + entryNotification.value.getValue());
          arcadeThrottleRamp = new SlewRateLimiter(1f / entryNotification.value.getDouble());
        },
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    turnRamp = Shuffleboard.getTab("Drive")
        .add("Turn Ramp", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.010, "max", 2.000))
        .getEntry();

    turnRamp.setDouble(Constants.DEFAULT_MAX_DEMAND_CHANGE);
    turnRamp.addListener(
        (entryNotification) -> {
          System.out.println("Turn Ramp changed value: " + entryNotification.value.getValue());
          arcadeTurnRamp = new SlewRateLimiter(1f / entryNotification.value.getDouble());
        },
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(imu.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field2d.setRobotPose(odometry.getPoseMeters());

    // Update the motor safety. This makes sure the system knows we're in control of
    // the motors.
    // If the code crashes, or somehow gets stuck in a loop, and this method isn't
    // called then
    // the motors will automaitcally stop.
    drive.feed();

  }

  /**
   * Called with every loop to update the simulation. periodic() is also called
   * as normal.
   */
  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    driveSim.setInputs(
        leftMotors.get() * RobotController.getInputVoltage(),
        rightMotors.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    driveSim.update(0.02);

    // Update all of our sensors.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    imuSim.getDouble("Yaw").set(-driveSim.getHeading().getDegrees());
  }

  /**
   * Normally, this should not be called, except form the setPose() method.
   * Using setPose() instead also updates the drive train, odometry, and drive
   * sim.bb
   */
  public void resetSensors() {
    leftEncoder.reset();
    rightEncoder.reset();
    imu.reset();
  }

  public void tankDrive(double left, double right) {
    System.out.println("Tank:left:" + left + " right:" + right);
    drive.tankDrive(
        left * reduction,
        right * reduction);
  }

  /**
   * Drive using arcade controls with speed ramping. Do not use for autonomous
   * routines
   * unless speed ramping is desired.
   */
  public void arcadeDrive(double throttle, double turn) {
    System.out.println("throttle:" + throttle + " turn:" + turn);
    double adjustedThrottle = arcadeThrottleRamp.calculate(throttle * reduction);
    double adjustedTurn = arcadeTurnRamp.calculate(turn * reduction);

    // System.out.println("Drive: arcadeDrive: ramped throttle: " + adjustedThrottle
    // + ", ramped turn: " + adjustedTurn);
    // Currently, applying factor to both throttle and turn, but we may want to
    // consider
    // only applying to throttle.
    drive.arcadeDrive(adjustedThrottle, adjustedTurn);
  }

  /**
   * Get the distance, in meters the left side has traveled since the last reset.
   */
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  /**
   * Get the distance, in meters the right side has traveled since the last reset.
   * 
   * @return
   */
  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public Field2d getField2d() {
    return field2d;
  }

  public void setPoseMeters(Pose2d pose) {
    field2d.setRobotPose(pose);
    imu.reset();
    odometry.resetPosition(pose, imu.getRotation2d());
    resetSensors();
    if (RobotBase.isSimulation()) {
      driveSim.setPose(pose);
    }
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public double getHeadingDegrees() {
    return imu.getAngle();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Gear", this::getGear, this::setGear);
    builder.addDoubleProperty("IMU roll", this::getRoll, null);
    builder.addDoubleProperty("IMU pitch", this::getPitch, null);
  }

  public double getRoll() {
    return imu.getPitch();
  }
// RIO IS MOUNTED SIDEWAYS. WE ARE NOT CRAZY.
  public double getPitch() {
    return imu.getRoll();
  }

  public double getGear() {
    return gear;
  }

  public void setGear(double g) {
    gear = (int) g;

    // Prevent the gear from going out of range
    // must be between 1 and the max gear, which is the number of items in the
    // GEAR_REDUCTION array
    int maxGear = GEAR_REDUCTION.length;
    if (gear > maxGear) {
      gear = maxGear;
    } else if (gear < 1) {
      gear = 1;
    }

    reduction = GEAR_REDUCTION[gear - 1];
  }

  public void shift(boolean up) {
    if (up) {
      gear++;
    } else {
      gear--;
    }

    // Prevent the gear from going out of range
    // must be between 1 and the max gear, which is the number of items in the
    // GEAR_REDUCTION array
    int maxGear = GEAR_REDUCTION.length;
    if (gear > maxGear) {
      gear = maxGear;
    } else if (gear < 1) {
      gear = 1;
    }

    reduction = GEAR_REDUCTION[gear - 1];
  }

  /**
   * Get the range to the closest object from the left sensor.
   * 
   * @return The range, in millimeters
   */
  public double getLeftSensorRange() {
    return rangeLeft.getRange();
  }

  /**
   * Get the range to the closest object from the right sensor.
   * 
   * @return The range, in millimeters
   */
  public double getRightSensorRange() {
    return rangeRight.getRange();
  }

  /**
   * Get the range to the closest object from the middle sensor.
   * 
   * @return The range, in millimeters
   */
  public double getMiddleSensorRange() {
    return rangeMiddle.getRange();
  }

  enum LevelingState {
    level_ground,
    climbing,
    level_energized,
    descending
  }

  public void energizePeriodic() {
    //is this the first time this is called?
      //Yes - record 'level' pitch
      //No - compare current pitch to level
        //did it go up, down, stay the same?
          //Up - set state to climbing - keep moving forward
          //Same
            //has it changed and went back to level? (is it in climbing state?)
              //No - no action
              //yes - stop moving
          //Down - start moving backwards
        
   // LevelingState == level_ground

    



      
    }
    
  
    









   //Accelerometer roborioAccelerometer =  new BuiltInAccelerometer();
   



  }


