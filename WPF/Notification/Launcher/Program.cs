using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Launcher
{
    class Program
    {
        static void Main(string[] args)
        {
            int interval = 10000;

            foreach (string arg in args)
            {
                if (arg.StartsWith("timeout:"))
                {
                    interval = Convert.ToInt32(arg.Substring(8)) * 1000;
                }
            }

			System.Threading.Thread.Sleep(900000);
			while (true)
            {
                System.Diagnostics.Process process = new System.Diagnostics.Process();
                System.Diagnostics.ProcessStartInfo startInfo = new System.Diagnostics.ProcessStartInfo();
                startInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
                startInfo.FileName = "cmd.exe";
                //startInfo.Arguments = "Notification.exe";
                startInfo.Arguments = "/C Notification.exe";
                process.StartInfo = startInfo;
                process.Start();

				System.Threading.Thread.Sleep(interval);
			}
        }
    }
}
