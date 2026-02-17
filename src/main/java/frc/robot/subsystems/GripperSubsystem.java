package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

    // הגדרת המנוע
    private final SparkMax sparkGripper
;

    // קבועים (מומלץ להעביר ל-Constants.java בהמשך)
    private static final int MOTOR_ID = 5; // וודא שזה ה-ID הנכון
    private static final int CURRENT_LIMIT = 30; // הגבלת זרם באמפר
    private static final double GRIP_SPEED = 0.5; // מהירות תפיסה
    private static final double EJECT_SPEED = -0.5; // מהירות שחרור

    public GripperSubsystem() {
        // אתחול המנוע
        sparkGripper = new SparkMax(MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        // הגדרת קונפיגורציה
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(CURRENT_LIMIT); // הגנה משריפת המנוע
        config.idleMode(SparkBaseConfig.IdleMode.kBrake); // עוצר מיד כשעוזבים כפתור
        config.inverted(false); // שנה ל-true אם המנוע מסתובב הפוך

        // החלת ההגדרות (חשוב מאוד!)
        sparkGripper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // פעולה להפעלת המנוע (מקבלת מהירות)
    public void setSpeed(double speed) {
        sparkGripper.set(speed);
    }

    // פעולה לעצירת המנוע
    public void stop() {
        sparkGripper
.set(0);
    }

    // Helper Methods לפקודות (Inline Commands)
    // שיטה זו מאפשרת ליצור פקודות ישירות ב-RobotContainer בצורה נקייה

    // פקודה לתפוס
    public void grip() {
        setSpeed(GRIP_SPEED);
    }

    // פקודה לשחרר
    public void eject() {
        setSpeed(EJECT_SPEED);
    }
}