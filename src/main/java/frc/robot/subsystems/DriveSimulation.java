package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.config.DriveConfig;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;


public class DriveSimulation {
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;

  private AnalogGyroSim gyroSim;
  private DifferentialDrivetrainSim driveSim;

  public DriveSimulation(
    Encoder leftEncoder,
    Encoder rightEncoder,
    AnalogGyro gyro
  ) {

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    gyroSim = new AnalogGyroSim(gyro);

    driveSim = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(DriveConfig.kvLinear, DriveConfig.kaLinear, DriveConfig.kvAngular, DriveConfig.kaAngular, DriveConfig.TrackWidthInMeters),
      DCMotor.getNEO(3),
      DriveConfig.GearingReduction,
      DriveConfig.TrackWidthInMeters,
      DriveConfig.WheelRadiusInMeters,
      DriveConfig.EnocoderNoise
    );
  }

  public void update(double leftSpeed, double rightSpeed) {

    driveSim.setInputs(
      leftSpeed * RobotController.getInputVoltage(),
      rightSpeed * RobotController.getInputVoltage()
    );

    driveSim.update(0.02);

    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());

    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }
}
