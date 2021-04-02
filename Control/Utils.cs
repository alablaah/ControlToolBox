using MathNet.Numerics.LinearAlgebra;

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
    }
}
