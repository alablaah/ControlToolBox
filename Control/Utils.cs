using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;

namespace ControlToolBox
{
    public class Utils
    {        
        public static Matrix<double> SubUpperRight(Matrix<double> Matrix, int Split)
        {
            int order = Matrix.ColumnCount;

            if (Split > order - 1)
            {
                throw new InvalidDimensionsException("Split is larger than matrix dimensions.");
            }

            return Matrix.SubMatrix(
                rowIndex: 0, rowCount: Split, columnIndex: order - Split, columnCount: Split
                );
        }

        public static bool CheckRank(Matrix<double> M, int Rank)
        {
            if (M.Rank() == Rank) { return true; }
            return false;
        }

        public static bool IsEqualToIdentity(Matrix<double> Matrix)
        {
            int order = Matrix.ColumnCount;
            if (Matrix.ColumnCount != Matrix.RowCount)
            {
                throw new InvalidDimensionsException("Matrix not square.");
            }
            return Matrix.Equals(Matrix<double>.Build.DenseIdentity(order: order));
        }

        public static bool IsZero(Vector<double> Vector)
        {
            return Vector.Equals(Vector<double>.Build.Dense(size: Vector.Count));
        }

        public static bool IsZero(Matrix<double> Matrix)
        {
            return Matrix
                .Equals(
                Matrix<double>.Build.Dense(
                    rows: Matrix.RowCount, columns: Matrix.ColumnCount, value: 0
                    )
                );
        }

        public class Timeseries
        {
            // TODO: Use Dict of Timestamped elements?
            public List<Vector<double>> Series { get; private set; }
            public string Name { get; private set; }
            public string Unit { get; private set; }
            public int Size { get; private set; }

            private int counter = 0;

            public Timeseries(string Name, string Unit, int Size, int? length = null)
            {
                this.Name = Name;
                this.Unit = Unit;
                this.Size = Size;

                if (length != null)
                {
                    Series = new List<Vector<double>>((int)length);
                }
                else
                {
                    Series = new List<Vector<double>>();
                }
            }

            public void AddElement(Vector<double> Element)
            {
                if (Element.Count == this.Size)
                {
                    Series.Add(Element);
                }
                else
                {
                    throw new InvalidDimensionsException($"Timeseries {this.Name} expects elements of length {this.Size}.");
                }
            }

            public Vector<double> GetElement(int index)
            {
                return Series[index];
            }

            public Vector<double> GetElementAndIncrement()
            {
                counter++;
                return GetElement(index: counter - 1);
            }

            public void ResetCounter()
            {
                counter = 0;
            }
        }
    }
}
