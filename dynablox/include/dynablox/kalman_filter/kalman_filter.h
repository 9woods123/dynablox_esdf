#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <stdexcept>

class KalmanFilter {
public:

    KalmanFilter(double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R, const Eigen::MatrixXd& P)
        : A(A), B(B), H(H), Q(Q), R(R), P0(P),
          m(A.rows()), n(B.cols()), dt(dt), initialized(false),
          I(m, m), x_hat(m), x_hat_new(m) {
    I.setIdentity();
    }
    

    void setMotionModel(const Eigen::MatrixXd& A_input, const Eigen::MatrixXd& B_input)
    {
        A=A_input;
        B=B_input;
    }


    void init() {
        x_hat.setZero();
        P = P0;
        initialized = true;
    }

    void init(double t0, const Eigen::VectorXd& x0) {
        x_hat = x0;
        P = P0;
        this->t0 = t0;
        t = t0;
        initialized = true;
    }


    bool update(const Eigen::VectorXd& y, const Eigen::VectorXd& u) {
        
        if (!initialized)
        {
            // throw std::runtime_error("Filter is not initialized!");
            return false;
        }

        // predict
        x_hat_new = A * x_hat + B * u;
        P = A * P * A.transpose() + Q;

        // kalman gain
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // update
        x_hat_new += K * (y - H * x_hat_new);
        P = (I - K * H) * P;

        x_hat = x_hat_new;

        t += dt;

        return true;
    }


    Eigen::VectorXd state() const { return x_hat; }
    double time() const { return t; }

private:
    Eigen::MatrixXd A, B, H, Q, R, P, K, P0;
    double t0, t, dt;
    int m, n;
    bool initialized;
    Eigen::MatrixXd I;
    Eigen::VectorXd x_hat, x_hat_new;

};

#endif // KALMAN_FILTER_H
