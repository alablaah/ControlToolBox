using System;
using System.Collections.Generic;
using Control;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace Test
{
    class TestProgram
    {
        static void Main(string[] args)
        {
            // create matrices for testing
            double m = 10; // kg
            double k = 2;  // N/m
            double f = 3;  // N/m/s

            double[,] a = {{0.0 , 1.0 },
                           {-k/m , -f/m}};

            double[,] b = {{ 1.0},
                           { 0.0 }};
            double[,] c = { { 1.0, 0.0 },
                            { 0.0, 1.0 } };
            double[,] d = {{ 0.0},
                           { 0.0 }};

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
            StateSpace Model = new StateSpace(A, B, C, D, x0: Vector<double>.Build.Dense(length: 2, value: 0));

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
            Model.ContinuousToDiscrete(Ts: 0.1, type: "FwdEuler");
            Model.PrintDiscreteModel();
            Console.WriteLine("C2D success.");

            Model.PrintState();

            //Vector<double> input = Vector<double>.Build.Dense(length: 1, value: 1);

            //for (int i = 0; i < 10; i++)
            //{
            //    Console.WriteLine(Model.SimulateStep(u_k: input));
            //}

            //Model.PrintState();

        }
    }
}
