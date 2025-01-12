/*package frc.utils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkPIDWrapper {
    private SparkMaxConfig sparkPID;

    SparkPIDWrapper(SparkMaxConfig sparkPID) {
        this.sparkPID = sparkPID;
    }

    public SparkMaxConfig getSparkPIDController() {
        return this.sparkPID;
    }

    public void setPID(double pGain, double iGain, double dGain, double fGain){
            this.sparkPID.closedLoop.pidf(pGain,iGain,dGain,fGain);
    }

    //TODO: I GENIUELNY DON"T KNOW WHY THERE IS A FOR LOOP AND IF STATEMENT, ESPECIALLY WHEN THE IF STATEMENT IMMEDITALY BREAKS
    // public void setP(double gain) {

    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setP(gain);
    //         if (this.sparkPID.getP() == gain) {
    //             break;
    //         }
    //     }
    // }

    // public void setI(double gain) {
    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setI(gain);
    //         if (this.sparkPID.getI() == gain) {
    //             break;
    //         }
    //     }
    // }

    // public void setD(double gain) {
    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setD(gain);
    //         if (this.sparkPID.getD() == gain) {
    //             break;
    //         }
    //     }

    // }

    // public void setIZone(double IZone) {
    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setIZone(IZone);
    //         if (this.sparkPID.getIZone() == IZone) {
    //             break;
    //         }
    //     }
    // }

    // public void setFF(double gain) {
    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setFF(gain);
    //         if (this.sparkPID.getFF() == gain) {
    //             break;
    //         }
    //     }
    // }

    public void setOutputRange(double min, double max) {
        this.sparkPID.closedLoop.outputRange(min, max);
    }

    // public void setSmartMotionMaxVelocity(double maxVal, int slotID) {
    //     this.sparkPID.closedLoop.ma
        
    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setSmartMotionMaxVelocity(maxVal, slotID);
    //         if (this.sparkPID.getSmartMotionMaxVelocity(slotID) == maxVal) {
    //             break;
    //         }
    //     }
    // }

    // public void setSmartMotionMinOutputVelocity(double minVel, int slotID) {
    //     for (int i = 0; i <= 5; i++) {
    //         this.sparkPID.setSmartMotionMinOutputVelocity(minVel, slotID);
    //         if (this.sparkPID.getSmartMotionMinOutputVelocity(slotID) == minVel) {
    //             break;
    //         }
    //     }
    // }

    public void setSmartMotionMaxAccel(double maxAccel, int slotID) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setSmartMotionMaxAccel(maxAccel, slotID);
            if (this.sparkPID.getSmartMotionMaxAccel(slotID) == maxAccel) {
                break;
            }
        }
    }

    public void setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
            if (this.sparkPID.getSmartMotionAllowedClosedLoopError(slotID) == allowedErr) {
                break;
            }
        }
    }

    public void setPositionPIDWrappingEnabled(boolean enable) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setPositionPIDWrappingEnabled(enable);
            if (this.sparkPID.getPositionPIDWrappingEnabled() == enable) {
                break;
            }
        }
    }

    public void setPositionPIDWrappingMinInput(double min) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setPositionPIDWrappingMinInput(min);
            if (this.sparkPID.getPositionPIDWrappingMinInput() == min) {
                break;
            }
        }
    }

    public void setPositionPIDWrappingMaxInput(double max) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setPositionPIDWrappingMaxInput(max);
            if (this.sparkPID.getPositionPIDWrappingMaxInput() == max) {
                break;
            }
        }
    }
}
*/