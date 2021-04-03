using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace ControlToolBox
{
    public class Simulation
    {
        public static List<Vector<double>> StepResponse(StateSpace model, double? Ts = null)
        {
            List<Vector<double>> Input = new List<Vector<double>>();
            List<Vector<double>> Output = new List<Vector<double>>();
            if (model.Tsample == null & Ts == null)
            {
                throw new SamplingRateRequiredException("No sampling time provided.");
            }

            else if (model.Tsample != null)
            {
                double Tsample = (double)model.Tsample;
            }
            else // Use Ts to discretize model
            {
                model.ContinuousToDiscrete(Ts: (double)Ts);
            }

            return Output;
        }
    }
}
