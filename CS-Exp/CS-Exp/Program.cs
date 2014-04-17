using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CS_Exp
{
    class Program
    {
        static void Main(string[] args)
        {
            //func1();
            func2();

        }

        /// <summary>
        /// List Examples
        /// </summary>
        static void func2()
        {
            List<string> list = new List<string>();

            list.Add("STR1");
            list.Add("STR2");
            list.Add("STR3");

            foreach(string str in list){
                Console.WriteLine(str);
            }

            System.Console.Read();
        }

        static void func1()
        {
            string str = "0101";

            System.Console.WriteLine(str[0]);
            System.Console.WriteLine(str[1]);
            System.Console.WriteLine(str[2]);
            System.Console.WriteLine(str[3]);

            System.Console.WriteLine(System.String.Format("{0}{1}{2}{3}", str[0], str[1], str[2], str[3]));
            System.Console.Read();
        }
    }
}
