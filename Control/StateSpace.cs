using System;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace ControlToolBox
{
    // TODOs
    // Add logging
    // Add KalmanFilter
    public class StateSpace
    {
        // System dynamical characteristics
        public Matrix<double> A { get; private set; }
        public Vector<System.Numerics.Complex> EigenValues { get; private set; }
        public Matrix<double> EigenVectors { get; private set; }
        public Vector<double> TfCoeff_a { get; private set; }
        public Matrix<double> TfCoeff_b { get; private set; }

        // System characteristics
        Matrix<double> I; //Identity matrix with shape of A
        public int Order { get; private set; }

        Matrix<double> R; // Controllability Gramian
        Matrix<double> W; // Observability Gramian

        // System matrices
        public Matrix<double> B { get; private set; }
        public Matrix<double> C { get; private set; }
        public Matrix<double> D { get; private set; }

        public Matrix<double> Ad { get; private set; }
        public Matrix<double> Bd { get; private set; }
        public Matrix<double> Cd { get; private set; }
        public Matrix<double> Dd { get; private set; }

        // State and output vectors
        public Vector<double> x_k { get; private set; }
        public Vector<double> x_knext { get; private set; }
        public Vector<double> y_k { get; private set; }

        // AD conversion
        public double? Tsample { get; private set; }


        // Init functions
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

            Order = this.A.ColumnCount;
            Evd<double> decomposition = this.A.Evd();
            EigenValues = decomposition.EigenValues;
            EigenVectors = decomposition.EigenVectors;
            I = Matrix<double>.Build.DenseIdentity(order: Order);

        }

        private void InitStateVector(Vector<double> x0)
        {
            if (x0 == null)
            {
                x_knext = Vector<double>.Build.Dense(size: Order);
            }
            else
            {
                if (x0.Count != Order)
                {
                    throw new InvalidDimensionsException("Init state vector and A matrix dimensions do not match.");

                }
                else
                {
                    x_knext = x0;
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
            else if (discrete & Ts == null & Ts == 0)
            {
                throw new SamplingRateRequiredException("Please provide a sampling rate or step size.");
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

        // AD conversion functions
        private double CheckAliasing(double coeff = 0.1)
        {

            double lambda_max = EigenValues.AbsoluteMaximum().Magnitude;
            return coeff / lambda_max;
        }

        private Matrix<double> CalculateS(double h)
        {
            // first term of sum
            Matrix<double> S = Matrix<double>.Build.DiagonalIdentity(Order) * h;

            for (int n = 2;  n <= Order; n++)
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

        private void VerletDiscretization(double Ts)
        {
            //Based on LeapFrog method: sympltecic!
        }

        private void DiscretizationMethodSwitch(string type, double Ts)
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
                throw new AliasingException("Sampling rate too low for given model");
            }

            Tsample = Ts;

            DiscretizationMethodSwitch(type, Ts);

            Cd = C;
            Dd = D;
        }

        // Model characteristics checks
        public Matrix<double> ComputeControllabilityMatrix()
        {
            // first term of sum
            Matrix<double> R = B;

            for (int n = 2; n <= Order; n++)
            {
                Matrix<double> Aexp = A.Power(exponent: (n - 1));
                R = R.Append(Aexp * B);
            }
            return R;
        }

        public Matrix<double> ComputeObservabilityMatrix()
        {
            Matrix<double> W = C.Transpose();
            for (int n = 2; n <= Order; n++)
            {
                Matrix<double> Aexp = A.Power(exponent: (n - 1));
                W = W.Append((C * Aexp).Transpose());
            }
            return W.Transpose();
        }

        public bool IsControllable()
        {
            R = ComputeControllabilityMatrix();
            return Utils.CheckRank(R, Rank: Order);
        }

        public bool IsObservable()
        {
            W = ComputeObservabilityMatrix();
            return Utils.CheckRank(W, Rank: Order);
        }

        public bool IsStable()
        {
            foreach (double element in EigenValues.Real())
            {
                if (element > 0)
                {
                    return false;
                }
            }
            return true;
        }

        public bool IsInFirstCompanionForm()
        {
            Matrix<double> AUpperRight = Utils.SubUpperRight(Matrix: A, Split: Order - 1);

            bool IsAFirstColZero = Utils.IsZero(A.Column(0).SubVector(index: 0, count: Order - 1));
            bool IsBFirstElementsZero = Utils.IsZero(
                B.SubMatrix(
                    rowIndex: 0, rowCount: Order - 1, columnIndex: 0, columnCount: B.ColumnCount
                    )
                );
            
            if (!IsAFirstColZero || !Utils.IsEqualToIdentity(AUpperRight) || !IsBFirstElementsZero)
            {
                return false;
            }

            return true;
        }

        // StateSpace to Transfer Function
        public void ComputeTFDenominatorCoeff()
        {
            // Assumes First CompanionForm!!
            Vector<double> Coeff_a = Vector<double>.Build.Dense(size: Order + 1);
            Coeff_a[0] = 1;

            for (int i = 1; i <= Order; i++)
            {
                Coeff_a[i] = -1 * A[Order-1, Order - i];
            }

            TfCoeff_a = Coeff_a;
        }

        public void ComputeTFNominatorCoeff()
        {
            if (D.ColumnCount > 1)
            {
                throw new InvalidDimensionsException("SS2TF currently only supported for Single Input systems.");
            }
            Matrix<double> Coeff_b = Matrix<double>.Build.Dense(rows: C.RowCount, columns: Order);

            for (int i = 0; i < C.RowCount; i++)
            {
                for (int j = 0; j < Order; j++)
                {
                    Coeff_b[i, j] = C[i, Order - j - 1] + TfCoeff_a[j + 1] * D[i, 0];
                }
            }

            TfCoeff_b = Coeff_b;
        }

        public void DetermineTFCoefficients()
        {
            if (!IsInFirstCompanionForm())
            {
                throw new InvalidDimensionsException(
                    "System not in First Companion Form. Can't compute TF coefficients directly.");
            }
            // Apply coefficients formula

            ComputeTFDenominatorCoeff();
            ComputeTFNominatorCoeff();
        }

        // Time domain stuff - simulation

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
                throw new SamplingRateRequiredException("Please provide a sampling rate.");
            }

            else if (Ts != null) // Recalibrate discrete model to given Ts
            {
                ContinuousToDiscrete((double)Ts);
            }

            RunModelStep(u_k);

            return y_k;
        }



        // Printing function
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
