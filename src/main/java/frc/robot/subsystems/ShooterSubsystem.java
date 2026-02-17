
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

    public class ShooterSubsystem extends SubsystemBase {


        private final TalonFX shooterTalon = new TalonFX(ShooterConstants.kShooterMotorPort);

        private final DutyCycleOut m_output = new DutyCycleOut(0);

        public ShooterSubsystem() {

        }

        public void spin() {
            shooterTalon.setControl(m_output.withOutput(ShooterConstants.kShooterSpeed));
        }


        public void reverse() {
            shooterTalon.setControl(m_output.withOutput(-ShooterConstants.kShooterSpeed));
        }

        public void stop() {
            shooterTalon.stopMotor();
        }
    }


