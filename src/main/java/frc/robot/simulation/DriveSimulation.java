package frc.robot.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.config.subsystems.DriveConfig;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;

public class DriveSimulation {
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;

  private ADIS16470_IMUSim gyroSim;
  private DifferentialDrivetrainSim driveSim;

  public DriveSimulation(
    Encoder leftEncoder,
    Encoder rightEncoder,
    ADIS16470_IMU gyro
  ) {

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);

    gyroSim = new ADIS16470_IMUSim(gyro);

    driveSim = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(DriveConfig.kvLinear, DriveConfig.kaLinear, DriveConfig.kvAngular, DriveConfig.kaAngular, DriveConfig.trackWidthInMeters),
      DCMotor.getNEO(3),
      DriveConfig.gearingReduction,
      DriveConfig.trackWidthInMeters,
      DriveConfig.wheelRadiusInMeters,
      DriveConfig.enocoderNoise
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

    gyroSim.setGyroAngleZ(driveSim.getHeading().getDegrees());
  }
}
