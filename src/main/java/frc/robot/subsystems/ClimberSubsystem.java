// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.subsystems.ClimberConfig;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftArmMotor = new WPI_TalonFX(CanBusId.LeftClimberMotor);
  private final WPI_TalonFX rightArmMotor = new WPI_TalonFX(CanBusId.RightClimberMotor);

  private TalonFXSimCollection leftArmSim;
  private TalonFXSimCollection rightArmSim;

  private final DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftArmForward, PneumaticChannel.LeftArmReverse);
  private final DoubleSolenoid rightArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightArmForward, PneumaticChannel.RightArmReverse);

  private boolean isClimberEnabled = false;

  public ClimberSubsystem() {
    leftArmSolenoid.set(ClimberConfig.initialArmPosition);
    rightArmSolenoid.set(ClimberConfig.initialArmPosition);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    leftArmMotor.configForwardSoftLimitThreshold(ClimberConfig.armExtensionLimit, 30);
    leftArmMotor.configReverseSoftLimitThreshold(ClimberConfig.armRetractionLimit, 30);
    leftArmMotor.configForwardSoftLimitEnable (true, 30);
    leftArmMotor.configReverseSoftLimitEnable(true, 30);

    rightArmMotor.configForwardSoftLimitThreshold(ClimberConfig.armExtensionLimit, 30);
    rightArmMotor.configReverseSoftLimitThreshold(ClimberConfig.armRetractionLimit, 30);
    rightArmMotor.configForwardSoftLimitEnable (true, 30);
    rightArmMotor.configReverseSoftLimitEnable(true, 30);

    // TODO: We need to invert one of the motors, but I'm not sure which one yet.
    // leftArmMotor.setInverted(TalonFXInvertType.Clockwise);
    // or rightArmMotor.setInverted(TalonFXInvertType.Clockwise);

    if (RobotBase.isSimulation() == true) {
      leftArmSim = leftArmMotor.getSimCollection();
      rightArmSim = rightArmMotor.getSimCollection();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climber Enabled", isClimberEnabled);
    SmartDashboard.putString("Left Arm Position", leftArmSolenoid.get() == Value.kForward ? "Forward" : "Reverse");
    SmartDashboard.putString("Right Arm Position", rightArmSolenoid.get() == Value.kForward ? "Forward" : "Reverse");
    SmartDashboard.putNumber("Left Arm Length", 100 * leftArmMotor.getSensorCollection().getIntegratedSensorPosition()/ClimberConfig.armExtensionLimit);
    SmartDashboard.putNumber("Right Arm Length", 100 * rightArmMotor.getSensorCollection().getIntegratedSensorPosition()/ClimberConfig.armExtensionLimit);
  }

  public void simulationPeriodic() {

    int ticksPerPeriodic = 1;

    double currentLeftArmPosition = leftArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newLeftArmPosition = (int) (currentLeftArmPosition + leftArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    double currentRightArmPosition = rightArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newRightArmPosition = (int) (currentRightArmPosition + rightArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    leftArmSim.setIntegratedSensorRawPosition(newLeftArmPosition);
    rightArmSim.setIntegratedSensorRawPosition(newRightArmPosition);

  }

  public void toggleClimberEnabled() {
    isClimberEnabled = !isClimberEnabled;
  }

  public void toggleLeftArmPosition() {
    if (isClimberEnabled){
      leftArmSolenoid.toggle();
    }
  }

  public void toggleRightArmPosition() {
    if (isClimberEnabled){
      rightArmSolenoid.toggle();
    }
  }

  public void extendLeftArm() {
    if (isClimberEnabled) {
      leftArmMotor.set(ClimberConfig.extendArmSpeed);
    }
  }

  public void retractLeftArm() {
    if (isClimberEnabled) {
      leftArmMotor.set(ClimberConfig.retractArmSpeed);
    }
  }

  public void stopLeftArm() {
    leftArmMotor.set(0);
  }

  public void extendRightArm() {
    if (isClimberEnabled) {
      rightArmMotor.set(ClimberConfig.extendArmSpeed);
    }
  }

  public void retractRightArm() {
    if (isClimberEnabled){
      rightArmMotor.set(ClimberConfig.retractArmSpeed);
    }
  }

  public void stopRightArm() {
    rightArmMotor.set(0);
  }
}
