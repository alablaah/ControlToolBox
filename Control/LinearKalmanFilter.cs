using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace ControlToolBox
{
    class LinearKalmanFilter : StateSpace
    {
        public Matrix<double> K { get; private set; }
        public Matrix<double> P { get; private set; }
        public Matrix<double> Q { get; private set; }
        public Matrix<double> R { get; private set; }
        public Vector<double> x_knext_est { get; private set; }
        public Matrix<double> P_est { get; private set; }

        public LinearKalmanFilter(
            Matrix<double> A, Matrix<double> B, Matrix<double> C, Matrix<double> D,
            Matrix<double> R, Matrix<double> Q, Matrix<double> P0,
            double Ts, Vector<double> x0
            ) : 
            base(A, B, C, D, Ts, x0, discrete : false)
        {
            this.R = R;
            this.Q = Q;
            this.P = P0;

            UpdateKalmanGain();
        }

        public void UpdateKalmanGain()
        {
            K = P_est * Cd.Transpose() * (Cd * P_est * Cd.Transpose() + R).Inverse();
        }
        public void ModelMeasurementFusion(Vector<double> yk)
        {
            x_knext = x_knext_est + K * (yk - Cd * x_knext_est);
        }

        public void UpdateErrorCovariance()
        {
            P_est = Ad * P * Ad.Transpose() + Q;
        }



    }
}
