/*
 * PDController.h
 *
 * Generic PD controller (proportional + derivative).
 * Usage:
 *   PDController pd(kp, kd);
 *   double output = pd.compute(desired, actual, dt);
 *
 * The output can be used to correct a setpoint:
 *   corrected_setpoint = desired + output
 *
 * Convention:
 *   error      = desired - actual
 *   derivative = (error - prev_error) / dt
 *   output     = Kp * error + Kd * derivative
 *
 * A positive output means "increase the controlled variable".
 */

#ifndef PD_CONTROLLER_H_
#define PD_CONTROLLER_H_

class PDController {
public:
    PDController(double kp, double kd)
        : kp_(kp), kd_(kd), prev_error_(0.0), initialized_(false) {}

    /**
     * Compute PD output.
     * @param desired   Target value.
     * @param actual    Measured value.
     * @param dt        Time step in seconds.
     * @return          Correction term.
     */
    double compute(double desired, double actual, double dt) {
        double error = desired - actual;

        double derivative = 0.0;
        if (initialized_ && dt > 1e-9) {
            derivative = (error - prev_error_) / dt;
        }

        prev_error_ = error;
        initialized_ = true;

        return kp_ * error + kd_ * derivative;
    }

    void setGains(double kp, double kd) {
        kp_ = kp;
        kd_ = kd;
    }

    void reset() {
        prev_error_ = 0.0;
        initialized_ = false;
    }

private:
    double kp_;
    double kd_;
    double prev_error_;
    bool   initialized_;
};

#endif  // PD_CONTROLLER_H_
