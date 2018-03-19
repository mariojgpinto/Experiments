using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace CS_Exp
{
    public class BoardState
    {
        public Vector2 move;
        public SolverBoard board;
        public int parent;
    }

    public class SolverClass
    {
        private ManualResetEvent _doneEvent;

        public string matrix;
        public int length;

        public int minMoves = -1;

        public bool solved = false;
        public int nextLevel = 1;
        public int currentLevel = 0;
        public List<List<BoardState>> statesByLevel = null;// new List<List<BoardState>>();

        public SolverClass(string _matrix, ManualResetEvent doneEvent)
        {
            matrix = _matrix;
            length = (int)Math.Ceiling(Math.Sqrt(_matrix.Length));

            statesByLevel = new List<List<BoardState>>();

            _doneEvent = doneEvent;
        }

        void AddLevel()
        {
            statesByLevel.Add(new List<BoardState>());
            currentLevel++;
            nextLevel++;
        }

        void AddBoardToSolve(SolverBoard b, int x, int y, int _parent)
        {
            statesByLevel[nextLevel].Add(new BoardState
            {
                move = new Vector2(x, y),
                board = b,
                parent = _parent
            });
        }

        public void Solve(object threadContext)
        {
            SolverBoard board = new SolverBoard(matrix);

            statesByLevel = new List<List<BoardState>>();
            statesByLevel.Add(new List<BoardState>());
            statesByLevel.Add(new List<BoardState>());
            nextLevel = 1;
            currentLevel = 0;

            statesByLevel[currentLevel].Add(new BoardState
            {
                move = new Vector2(0, 0),
                board = board,
                parent = -1
            });

            while (!solved)
            {
                DateTime t = DateTime.Now;
                for (int b = 0; b < statesByLevel[currentLevel].Count; ++b)
                {
                    SolverBoard currentBoard = statesByLevel[currentLevel][b].board;

                    for (int x = currentBoard.minX; x < currentBoard.maxX; ++x)
                    {
                        for (int y = currentBoard.minY; y < currentBoard.maxY; ++y)
                        {
                            SolverBoard aux = new SolverBoard(currentBoard);

                            aux.ApplyMove(x, y);

                            if (statesByLevel.Count > 2)
                            {
                                //int grandFather = statesByLevel[currentLevel][b].parent
                                bool exist = false;
                                for (int i = 0; i < statesByLevel.Count - 2 && !exist; ++i)
                                {
                                    for (int j = 0; j < statesByLevel[i].Count && !exist; ++j)
                                    {
                                        if (SolverBoard.Compare(aux, statesByLevel[i][j].board))
                                            exist = true;
                                    }
                                }

                                if (!exist)
                                    AddBoardToSolve(aux, x, y, b);
                            }
                            else
                            {
                                AddBoardToSolve(aux, x, y, b);
                            }

                            //AddBoardToSolve(aux, x, y, b);

                            if (aux.solved)
                            {
                                //PUZZLE SOLVED!!!
                                solved = true;
                            }
                        }
                    }
                }

                DateTime t2 = DateTime.Now;
                TimeSpan t3 = t2.Subtract(t);

                Console.WriteLine("Level " + (statesByLevel.Count - 1) + ": " + statesByLevel[statesByLevel.Count - 1].Count + " - " + ((t2 - t).Milliseconds / 1000f) + " seconds");

                if (!solved)
                {
                    AddLevel();
                }
            }

            minMoves = statesByLevel.Count - 1;

            SetEvent();

            //PrintResult();
        }

        void SetEvent()
        {
            if(_doneEvent != null)
                _doneEvent.Set();
        }

        public void PrintResult()
        {

            //for (int i = 0; i < statesByLevel.Count; ++i)
            //{
            //    Console.WriteLine("Level " + i + ": " + statesByLevel[i].Count);                
            //}

            Console.WriteLine("\n");

            int nSolutions = 0;
            for (int i = 0; i < statesByLevel[minMoves].Count; ++i)
            {
                if (statesByLevel[minMoves][i].board.solved)
                {
                    nSolutions++;

                    BoardState b = statesByLevel[minMoves][i];
                    int level = minMoves;
                    string solution = "";
                    while (b.parent != -1)
                    {
                        solution = "(" + b.move.x + "," + b.move.y + ")" + solution;
                        level--;
                        b = statesByLevel[level][b.parent];
                    }

                    Console.WriteLine("Solution " + (nSolutions) + ": " + solution);
                }
            }

            Console.WriteLine("Min Moves: " + minMoves);
            Console.WriteLine("NSolutions: " + nSolutions);
        }
    }
}
