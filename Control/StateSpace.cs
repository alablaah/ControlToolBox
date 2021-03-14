using System;
using System.Reactive;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace Control
{
    public class StateSpace
    {
        Matrix<double> A;
        Vector<System.Numerics.Complex> EigenValues;
        Matrix<double> EigenVectors;

        Matrix<double> I; //Identify matrix with shape of A
        int order;


        Matrix<double> B;
        Matrix<double> C;
        Matrix<double> D;

        Matrix<double> Ad;
        Matrix<double> Bd;
        Matrix<double> Cd;
        Matrix<double> Dd;

        Vector<double> x_k;
        Vector<double> x_knext;
        Vector<double> y_k;

        double? Tsample;
        
        private void CheckMatrixDimensions(
            Matrix<double> A, Matrix<double> B, Matrix<double> C, Matrix<double> D
            )
        {
            if (A.ColumnCount != A.RowCount) 
            { throw new InvalidDimensionsException("A matrix not square."); }

            if (B.RowCount != A.RowCount)
            { throw new InvalidDimensionsException("B and A matrix dimensions do not match."); }

            if (C.ColumnCount != A.RowCount)
            { throw new InvalidDimensionsException("C and A matrix dimensions do not match."); }

            if (D.RowCount != C.RowCount)
            { throw new InvalidDimensionsException("D and C matrix dimensions do not match."); }


        }

        private void InitContinuousModel(
            Matrix<double> A, Matrix<double> B, Matrix<double> C, Matrix<double> D
            )
        {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;

            order = this.A.ColumnCount;
            Evd<double> decomposition = this.A.Evd();
            EigenValues = decomposition.EigenValues;
            EigenVectors = decomposition.EigenVectors;
            I = Matrix<double>.Build.DenseIdentity(order: order);

        }

        private void InitStateVector(Vector<double> x0)
        {
            if (x0 == null)
            {
                x_k = Vector<double>.Build.Dense(size: order);
                x_knext = x_k;
            }
            else
            {
                if (x0.Count != order)
                {
                    throw new InvalidDimensionsException("Init state vector and A matrix dimensions do not match.");

                }
                else
                {
                    x_k = x0;
                    x_knext = x_k;
                }

            }
        }
        public StateSpace(
            Matrix<double> A, Matrix<double> B, Matrix<double> C, Matrix<double> D,
            double? Ts = null, Vector<double> x0 = null, bool discrete = false
            )
        {

            CheckMatrixDimensions(A, B, C, D);

            if (discrete & Ts != null & Ts != 0) // explicitly discrete matrices
            {
                Tsample = Ts;

                Ad = A;
                Bd = B;
                Cd = C;
                Dd = D;

                // Discr2Cont

            }
            else // not discrete matrices
            {
                InitContinuousModel(A, B, C, D);

                if (Ts != null & Ts != 0) // Also create discrete form
                {
                    ContinuousToDiscrete((double)Ts);
                }
            }

            InitStateVector(x0);

        }

        private double CheckAliasing(double coeff = 0.1)
        {

            double lambda_max = EigenValues.AbsoluteMaximum().Magnitude;
            return coeff / lambda_max;
        }

        private Matrix<double> CalculateS(double h)
        {
            // first term of sum
            Matrix<double> S = Matrix<double>.Build.DiagonalIdentity(order) * h;

            for (int n = 2;  n <= order; n++)
            {
                Matrix<double> Aexp = A.Power(exponent: (n - 1));
                S += Aexp * Math.Pow(h, n) / SpecialFunctions.Factorial(n);
            }
            return S;
        }

        private void ZeroOrderHoldApproximateDiscretization(double Ts)
        {
            Matrix<double> S = CalculateS(Ts);

            Ad = I + A * S;
            Bd = S * B;

        }

        private void ForwardEulerDiscretization(double Ts)
        {
            Ad = I + A * Ts;
            Bd = B * Ts;
        }

        private void TustinDiscretization(double Ts)
        {
            //Matrix<double> Inverse = (I - A * Ts / 2).Inverse();
            //Ad = Inverse * (I + A * Ts / 2);
            //Bd = Inverse * B * Ts / 2; // needs previous input!?
        }
        private void LaplaceBasedExactDiscratization(double Ts)
        {
            // How to do inverse Laplace transform?
        }

        private void MethodSwitch(string type, double Ts)
        {
            switch(type)
            {
                case "zoh":
                    ZeroOrderHoldApproximateDiscretization(Ts);
                    break;
                case "FwdEuler":
                    ForwardEulerDiscretization(Ts);
                    break;
                case "Tustin":
                    throw new NotImplementedException("Tustin method not yet implemented.");
                case "Analytical":
                    throw new NotImplementedException("Analytical method not yet implemented.");
            }
        }

        public void ContinuousToDiscrete(double Ts, string type = "zoh")
        {
            if (Tsample != null)
            {
                Console.WriteLine($"Warning! Overwriting existing sampling rate {this.Tsample}s with {Ts}s.");
            }
            if (Ts >= CheckAliasing())
            {
                throw new AliasingException("Sampling Rate too low for given model");
            }

            Tsample = Ts;

            MethodSwitch(type, Ts);

            Cd = C;
            Dd = D;
        }
        private void RunModelStep(Vector<double> u_k)
        {
            x_k = x_knext;
            x_knext = Ad * x_k + Bd * u_k;
            y_k = Cd * x_k + Dd * u_k;
        }
        public Vector<double> SimulateStep(Vector<double> u_k, double? Ts = null)
        {
            if (u_k.Count != this.B.ColumnCount)
            { throw new InvalidDimensionsException("control vector and B matrix dimensions do not match."); }

            if (Ts == null && Tsample == null)
            {
                // Throw error for not having a sampling time
            }

            else if (Ts != null) // Recalibrate discrete model to given Ts
            {
                ContinuousToDiscrete((double)Ts);
            }

            RunModelStep(u_k);

            return y_k;
        }
        public void PrintContinuousModel()
        {
            Console.WriteLine($"LTI-state-space model\n A:\n{A}\n\nB:\n{B}\n\nC:\n{C}\n\nD:\n{D}\n\n");
        }

        public void PrintDiscreteModel()
        {
            Console.WriteLine($"LTI-state-space model\n Ad:\n{Ad}\n\nBd:\n{Bd}\n\nCd:\n{Cd}\n\nDd:\n{Dd}\n\n");
        }

        public void PrintState()
        {
            Console.WriteLine($"Current State: {x_k}\nCurrent output: {y_k}");
        }

        public Matrix<double> GetAd()
        {
            return Ad;
        }
    }
}
