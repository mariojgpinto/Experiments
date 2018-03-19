using System;

namespace CS_Exp
{
    public class SolverBoard
    {
        public bool[] matrix;
        public int width, height;
        public int minX, maxX, minY, maxY;

        public bool solved = false;

        public SolverBoard(string _matrix)
        {
            matrix = new bool[_matrix.Length];
            for (int i = 0; i < _matrix.Length; matrix[i] = _matrix[i++] == '1') ;

            width = height = (int)Math.Ceiling(Math.Sqrt(_matrix.Length));

            minX = 0;
            minY = 0;
            maxX = width;
            maxY = height;

            UpdateLimits();
        }

        public SolverBoard(SolverBoard _board)
        {
            matrix = new bool[_board.matrix.Length];
            for (int i = 0; i < _board.matrix.Length; matrix[i] = _board.matrix[i++]) ;
            
            width = height = (int)Math.Ceiling(Math.Sqrt(matrix.Length));

            minX = 0;
            minY = 0;
            maxX = width;
            maxY = height;

            UpdateLimits();
        }

        public void ApplyMove(int x, int y)
        {
            if (x < minX || x > maxX || y < minY || y > maxY)
                return;

            int idx = y * width + x;

            matrix[idx] = !matrix[idx];
            for (int i = minX; i < maxX; ++i)
            {
                idx = width * y + i;

                matrix[idx] = !matrix[idx];

            }

            for (int j = minY; j < maxY; ++j)
            {
                idx = width * j + x;
                matrix[idx] = !matrix[idx];
            }

            UpdateLimits();
        }

        public void UpdateLimits()
        {
            bool foundMinX = false;
            for (int x = minX; x < maxX && !foundMinX; ++x)
            {
                for (int y = minY; y < maxY && !foundMinX; ++y)
                {
                    if (matrix[width * y + x])
                    {
                        minX = x;
                        foundMinX = true;
                    }
                }
            }
            if (!foundMinX)
            {
                minX = maxY;
                solved = true;
                return;
            }

            bool foundMinY = false;
            for (int y = minY; y < maxY && !foundMinY; ++y)
            {
                for (int x = minX; x < maxX && !foundMinY; ++x)
                {
                    if (matrix[width * y + x])
                    {
                        minY = y;
                        foundMinY = true;
                    }
                }
            }
            if (!foundMinY)
            {
                minY = maxY;
                solved = true;
                return;
            }

            bool foundMaxX = false;
            for (int x = maxX - 1; x >= minX && !foundMaxX; --x)
            {
                for (int y = minY; y < maxY && !foundMaxX; ++y)
                {
                    if (matrix[width * y + x])
                    {
                        maxX = x + 1;
                        foundMaxX = true;
                    }
                }
            }
            if (!foundMaxX)
            {
                maxX = minX;
                solved = true;
                return;
            }

            bool foundMaxY = false;
            for (int y = maxY - 1; y >= minY && !foundMaxY; --y)
            {
                for (int x = minX; x < maxX && !foundMaxY; ++x)
                {
                    if (matrix[width * y + x])
                    {
                        maxY = y + 1;
                        foundMaxY = true;
                    }
                }
            }
            if (!foundMaxY)
            {
                maxY = minY;
                solved = true;
                return;
            }
        }

        public string ToString()
        {
            string str = "";

            str = "Board (" + width + "," + height + "):\n";

            for (int j = height - 1; j >= 0; --j)
            {
                for (int i = 0; i < width; ++i)
                {
                    str += (matrix[j * width + i] ? "1" : "0") + ",";
                }
                str += "\n";
            }

            str += "BordersX[" + minX + "," + maxX + "]\n";
            str += "BordersY[" + minY + "," + maxY + "]\n";

            return str;
        }

        public static bool Compare(SolverBoard b1, SolverBoard b2)
        {
            if (b1.matrix.Length != b2.matrix.Length)
                return false;

            if (b1.minX != b2.minX ||
               b1.minY != b2.minY ||
               b1.maxX != b2.maxX ||
               b1.maxY != b2.maxY)
                return false;

            for (int i = 0; i < b1.matrix.Length; ++i)
            {
                if (b1.matrix[i] != b2.matrix[i])
                {
                    return false;
                }
            }
            return true;
        }
    }
}
