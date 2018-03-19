using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace CS_Exp
{
    class ThreadedSolver
    {
        public static void Main_ThreadedSolver1()
        {
            Thread T = new Thread(Main_ThreadedSolver1, 900000000);
            T.Start();
        }

        public static void Main_ThreadedSolver_oneThread() { 
            Console.WriteLine("Press:\n"+
                "  1 - 4x4 - 1110000101100100\n"+
                "  2 - 6x6 - 001101110010010100101100001000000010\n" +
                "  3 - 8x8 - 0111100011000010101100010001001100000010111101001010110000100001\n");

            int x = Console.Read();

            string m = "1110000101100100";
            if (x == '1')
            {
                m = "1110000101100100";
            } else
            if (x == '2')
            {
                m = "001101110010010100101100001000000010";
            }
            else
            if (x == '3')
            {
                m = "0111100011000010101100010001001100000010111101001010110000100001";
            }

            Console.WriteLine("\nSolving for [" + m + "]\n");

            SolverBoard b = new SolverBoard(m);

            Console.WriteLine(b.ToString());

            SolverClass solver = new SolverClass(m, null);
            solver.Solve(null);

            Console.WriteLine("\nPress any key to exit...");
            Console.Read();
            Console.Read();
            Console.Read();



            //int nThreads = m.Length;

            //// One event is used for each Fibonacci object.
            //ManualResetEvent[] doneEvents = new ManualResetEvent[nThreads];
            //SolverClass[] threadArray = new SolverClass[nThreads];

            //// Configure and start threads using ThreadPool.
            //Console.WriteLine("launching {0} tasks...", nThreads);

            //for (int i = 0; i < nThreads; i++)
            //{
            //    doneEvents[i] = new ManualResetEvent(false);
            //    //Fibonacci f = new Fibonacci(r.Next(20, 40), doneEvents[i]);

            //    SolverClass solver = new SolverClass(m, doneEvents[i]);
            //    threadArray[i] = solver;
            //    ThreadPool.QueueUserWorkItem(solver.Solve, i);
            //}

            //// Wait for all threads in pool to calculate.
            //WaitHandle.WaitAll(doneEvents);
            //Console.WriteLine("All calculations are complete.");

            //// Display the results.
            //for (int i = 0; i < nThreads; i++)
            //{
            //    SolverClass f = threadArray[i];
            //    //Console.WriteLine("Fibonacci({0}) = {1}", f.N, f.FibOfN);
            //}

            //Console.WriteLine("");
        }

        public static void Main_ThreadedSolver_Threaded()
        {
            Console.WriteLine("Press:\n" +
                "  1 - 4x4 - 1110000101100100\n" +
                "  2 - 6x6 - 001101110010010100101100001000000010\n" +
                "  3 - 8x8 - 0111100011000010101100010001001100000010111101001010110000100001\n");

            int x = 0;

            try {
                x = System.Convert.ToInt32(Console.ReadLine());
            }
            catch { }

            string m = "1110000101100100";
            if (x == 1)
            {
                m = "1110000101100100";
            }
            else
            if (x == 2)
            {
                m = "001101110010010100101100001000000010";
            }
            else
            if (x == 3)
            {
                m = "0111100011000010101100010001001100000010111101001010110000100001";
            }

            Console.WriteLine("\nSolving for [" + m + "]\n");

            SolverBoard b = new SolverBoard(m);

            Console.WriteLine(b.ToString());





            int nThreads = m.Length;

            // One event is used for each Fibonacci object.
            ManualResetEvent[] doneEvents = new ManualResetEvent[nThreads];
            SolverClass[] threadArray = new SolverClass[nThreads];

            // Configure and start threads using ThreadPool.
            Console.WriteLine("launching {0} tasks...", nThreads);

            for (int i = 0; i < nThreads; i++)
            {
                doneEvents[i] = new ManualResetEvent(false);

                SolverClass solver = new SolverClass(m, doneEvents[i]);
                threadArray[i] = solver;
                ThreadPool.QueueUserWorkItem(solver.Solve, i);
            }

            // Wait for all threads in pool to calculate.
            WaitHandle.WaitAny(doneEvents);
            Console.WriteLine("All calculations are complete.");

            // Display the results.
            for (int i = 0; i < nThreads; i++)
            {
                SolverClass f = threadArray[i];

                if (f.solved)
                {
                    f.PrintResult();
                    Console.WriteLine("");
                    break; 
                }
                //Console.WriteLine("Fibonacci({0}) = {1}", f.N, f.FibOfN);
            }

            Console.ReadLine();
        }
    }
}
