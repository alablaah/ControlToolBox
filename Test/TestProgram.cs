using System;
using ControlToolBox;
using MathNet.Numerics.LinearAlgebra;

namespace Test
{
    class TestProgram
    {
        static void Main(string[] args)
        {
            // create matrices for testing
            double m = 1; // kg
            double k = 2;  // N/m
            double f = 3;  // N/m/s

            double[,] a = {{0.0, 1.0 , 0.0 },
                           {0.0, 0.0 , 1.0 },
                           {1, -k/m , -f/m}};

            double[,] b = {{ 0.0},
                           { 0.0 },
                           { 1.0 } };

            //double[,] c = { { 0.0, 1.0, 0.0 } };

            double[,] c = { {0.0, 1.0, 0.0 },
                            {0.0,  0.0, 1.0 } };

            double[,] d = {{ 0},
                           { 0}};

            //double[,] a = {{0.0 , 1.0 },
            //               {-k/m , -f/m}};

            //double[,] b = {{ 0.0},
            //               { 1.0 }};


            //double[,] c = { { 1.0, 0.0 },
            //                { 0.0, 1.0 } };

            //double[,] c = { { 1.0, 0.0 }};

            //double[,] d = {{ 0.1}};

            Matrix<double> A = Matrix<double>.Build.DenseOfArray(a);
            Matrix<double> B = Matrix<double>.Build.DenseOfArray(b);
            Matrix<double> C = Matrix<double>.Build.DenseOfArray(c);
            Matrix<double> D = Matrix<double>.Build.DenseOfArray(d);

            // Test CheckDimensions
            try
            {
                Matrix<double> AFake = Matrix<double>.Build.Dense(rows: 2, columns: 3);
                StateSpace ModelFake = new StateSpace(AFake, B, C, D);
                Console.WriteLine("CheckDimensions failed.");
            }
            catch (InvalidDimensionsException)
            {
                Console.WriteLine("CheckDimensions success.");
            }

            // Test initialization
            StateSpace Model = new StateSpace(A, B, C, D, x0: Vector<double>.Build.Dense(length: A.ColumnCount, value: 0));

            Console.WriteLine("Model Initialization Continuous success.");

            // Test CheckAliasing
            try
            {
                Model.ContinuousToDiscrete(Ts: 1);
                Console.WriteLine("CheckAliasing failed.");
            }
            catch (AliasingException)
            {
                Console.WriteLine("CheckAliasing success.");
            }

            // Test C2D
            //Model.ContinuousToDiscrete(Ts: 0.1, type: "zoh");
            //Model.PrintDiscreteModel();
            //Console.WriteLine("C2D success.");

            //Model.PrintState();

            //Vector<double> input = Vector<double>.Build.Dense(length: 1, value: 1);

            //for (int i = 0; i < 10; i++)
            //{
            //    Console.WriteLine(Model.SimulateStep(u_k: input));
            //}

            //Model.PrintState();

            // Test RComputation

            if (Model.IsControllable())
            {
                Console.WriteLine("CheckControllability success.");
            }
            else
            {
                Console.WriteLine("CheckControllability failed.");
            }

            if (Model.IsObservable())
            {
                Console.WriteLine("CheckObservability success.");
            }
            else
            {
                Console.WriteLine("CheckObservability failed.");
            }

            Matrix<double> T = Matrix<double>.Build.RandomPositiveDefinite(order: 4);
            T[0, 0] = 0;
            T[1, 0] = 0;
            T[2, 0] = 0;
            //Console.WriteLine($"\n\nOriginal Matrix: \n\n{T}");
            //Console.WriteLine($"\n\nSplit 1 Matrix: \n\n{Utils.SubUpperRight(Matrix: T, Split: 1)}");
            //Console.WriteLine($"\n\nSplit 2 Matrix: \n\n{Utils.SubUpperRight(Matrix: T, Split: 2)}");
            //Console.WriteLine($"\n\nSplit 3 Matrix: \n\n{Utils.SubUpperRight(Matrix: T, Split: 3)}");
            //Console.WriteLine(T);
            //Console.WriteLine(Vector<double>.Build.Dense(size: 3));

            try
            {
                Console.WriteLine($"\n\nSplit 4 Matrix: \n\n{Utils.SubUpperRight(Matrix: T, Split: 4)}");
                Console.WriteLine("SubUpperRight failed.");
            }
            catch (InvalidDimensionsException)
            {
                Console.WriteLine("SubUpperRight success.");
            }

            if (Model.IsInFirstCompanionForm())
            {
                Console.WriteLine("FirstCompanionForm success.");
            }
            else
            Console.WriteLine($"\n\nIsFirstCompanionForm: \n\n{Model.IsInFirstCompanionForm()}");

            Model.DetermineTFCoefficients();

            double[] coeff_expected = { 1.0, 3.0, 2.0, -1.0 };
            if (Model.TfCoeff_a != Vector<double>.Build.DenseOfArray(coeff_expected))
            {
                Console.WriteLine("ComputeTfCoeff_a success.");
            }
            else
            {
                Console.WriteLine("ComputeTfCoeff_a failed.");
            }
            //Console.WriteLine($"den: {Model.TfCoeff_a}, num: {Model.TfCoeff_b}");

            Console.WriteLine(Model.IsStable());
        }


    }
}
