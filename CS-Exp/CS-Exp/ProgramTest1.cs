using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;

namespace CS_Exp
{
    class Container
    {
        public int a;
        public int b;
    }

    public class HiResDateTime
    {
        private static DateTime _startTime;
        private static Stopwatch _stopWatch = null;
        private static TimeSpan _maxIdle =
            TimeSpan.FromSeconds(10);

        public static DateTime UtcNow
        {
            get
            {
                if ((_stopWatch == null) ||
                    (_startTime.Add(_maxIdle) < DateTime.UtcNow))
                {
                    Reset();
                }
                return _startTime.AddTicks(_stopWatch.Elapsed.Ticks);
            }
        }

        private static void Reset()
        {
            _startTime = DateTime.UtcNow;
            _stopWatch = Stopwatch.StartNew();
        }
    }

    class ProgramTest1
    {
        static int ac_func5 = 0;

        public static string RemoveDiacritics(string text)
        {
            var normalizedString = text.Normalize(NormalizationForm.FormD);
            var stringBuilder = new StringBuilder();

            foreach (var c in normalizedString)
            {
                var unicodeCategory = CharUnicodeInfo.GetUnicodeCategory(c);
                if (unicodeCategory != UnicodeCategory.NonSpacingMark)
                {
                    stringBuilder.Append(c);
                }
            }

            return stringBuilder.ToString().Normalize(NormalizationForm.FormC);
        }

        public static void Main_ProgramTest1(string[] args)
        {
            ////Basic Examples
            //func1();

            ////Classes Examples
            //func2();

            ////Read and Write from other PC. (Not Working)
            //func3();

            ////Read and Write from other PC.
            //func4();

            //Serial Port
            //func5();

            //Random
            //func6();

            //FTP Upload
            //func7();

            //File Sniffer
            //func8();

            //Remove diatrics
            //func9();

            //Time
            func10();
        }

        static void func10()
        {
            DateTime start_time = DateTime.Now;

            System.Threading.Thread.Sleep(1150);

            DateTime end_time = DateTime.Now;

            Console.WriteLine((end_time - start_time).TotalMilliseconds);

            Console.ReadKey();
        }

        static void func9()
        {
            Console.WriteLine(RemoveDiacritics("Mário João") + "   |   " + "Mário João");

            System.IO.File.WriteAllText("Diatrics.txt", System.String.Format("With:{0}\nWithout:{1}", "Mário João", RemoveDiacritics("Mário João")));

            System.Threading.Thread.Sleep(2000);
        }

        static void func8()
        {
            string _directoryPath = "\\\\ILLUSTRATOR-1\\RockInRio\\Fotos\\";
            string localPath = "C:\\RockInRio\\Fotos2\\";

            string _userName = "Leonel Mendes";
            string _password = "01";

            Console.WriteLine("Disconnect in case we are currently connected with our credentials.");
            string result_disconnect = NetworkShare.DisconnectFromShare(_directoryPath, true); //Disconnect in case we are currently connected with our credentials;
            Console.WriteLine("Result:" + result_disconnect);

            Console.WriteLine("Connect with the new credentials.");
            string result_connect = NetworkShare.ConnectToShare(_directoryPath, _userName, _password); //Connect with the new credentials
            Console.WriteLine("Result:" + result_connect);

            while (true)
            {
                Console.WriteLine("Sniff");
                IEnumerable<string> files = Directory.EnumerateFiles(_directoryPath);

                foreach (string file in files)
                {
                    string[] parts = file.Split('\\');

                    string dest = localPath + parts[parts.Length - 1];

                    Console.WriteLine("\n\nFile Found: " + parts[parts.Length - 1] + "\nMoving File to: " + dest);

                    File.Copy(file, dest, true);
                    File.Delete(file);
                }

                System.Threading.Thread.Sleep(250);
            }
        }

        static void func7()
        {
            string filename = "Explosions1080.mp4";
            string ftpServerIP = "212.0.160.211/rock-street/2014-05-25/Jessie_J";///web/media/uploads/rock-street/2014-05-25/Jessie_J/
            string ftpUserName = "continente.havasrockinrio";
            string ftpPassword = "MM%s3V9$";

            FileInfo objFile = new FileInfo(filename);
            FtpWebRequest objFTPRequest;

            // Create FtpWebRequest object 
            objFTPRequest = (FtpWebRequest)FtpWebRequest.Create(new Uri("ftp://" + ftpServerIP + "/" + objFile.Name));

            // Set Credintials
            objFTPRequest.Credentials = new NetworkCredential(ftpUserName, ftpPassword);

            // By default KeepAlive is true, where the control connection is 
            // not closed after a command is executed.
            objFTPRequest.KeepAlive = false;

            // Set the data transfer type.
            objFTPRequest.UseBinary = true;

            // Set content length
            objFTPRequest.ContentLength = objFile.Length;

            // Set request method
            objFTPRequest.Method = WebRequestMethods.Ftp.UploadFile;

            // Set buffer size
            int intBufferLength = 16 * 1024;
            byte[] objBuffer = new byte[intBufferLength];

            // Opens a file to read
            FileStream objFileStream = objFile.OpenRead();

            Console.WriteLine("Time: " + HiResDateTime.UtcNow); ;

            try
            {
                // Get Stream of the file
                Stream objStream = objFTPRequest.GetRequestStream();

                int len = 0;

                while ((len = objFileStream.Read(objBuffer, 0, intBufferLength)) != 0)
                {
                    // Write file Content 
                    objStream.Write(objBuffer, 0, len);

                }

                objStream.Close();
                objFileStream.Close();
            }
            catch (Exception ex)
            {
                throw ex;
            }

            Console.WriteLine("Time: " + HiResDateTime.UtcNow); ;

            Console.Read();

            //// Get the object used to communicate with the server.
            //FtpWebRequest request = (FtpWebRequest)WebRequest.Create("ftp://212.0.160.211/");///web/media/uploads/rock-street/2014-05-25/Jessie_J/
            //request.Method = WebRequestMethods.Ftp.UploadFile;

            //// This example assumes the FTP site uses anonymous logon.
            //request.Credentials = new NetworkCredential("MM%s3V9$", "continente.havasrockinrio");

            //// Copy the contents of the file to the request stream.
            //StreamReader sourceStream = new StreamReader("teste.mp4");
            //byte[] fileContents = Encoding.UTF8.GetBytes(sourceStream.ReadToEnd());
            //sourceStream.Close();
            //request.ContentLength = fileContents.Length;

            //Stream requestStream = request.GetRequestStream();
            //requestStream.Write(fileContents, 0, fileContents.Length);
            //requestStream.Close();

            //FtpWebResponse response = (FtpWebResponse)request.GetResponse();

            //Console.WriteLine("Upload File Complete, status {0}", response.StatusDescription);

            //response.Close();
        }

        static void func6()
        {
            Random r = new Random();

            for (int i = 0; i < 20; ++i) {
                Console.WriteLine("R: " + r.Next(0, 6));
            }

            Console.Read();
        }

        static void func5()
        {
            SerialPort mySerialPort = new SerialPort("COM7");

            mySerialPort.BaudRate = 2400;
            mySerialPort.Parity = Parity.None;
            //mySerialPort.StopBits = StopBits.One;
            mySerialPort.DataBits = 8;
            mySerialPort.Handshake = Handshake.None;

            mySerialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

            mySerialPort.Open();

            Console.WriteLine("Press any key to continue...");
            Console.WriteLine();
            Console.ReadKey();
            mySerialPort.Close();
        }

        private static void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            string indata = sp.ReadExisting();

            string indata2 = indata.Replace("\r", " ");
            string indata3 = indata2.Replace("\n", " ");

            string[] indata4 = indata.Split(new char[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            if (indata3.Length > 0)
            {
                foreach (string s in indata4)
                {
                    Console.WriteLine(String.Format("-[{0}]-", s));
                }
                Console.WriteLine(String.Format("Data Received{0}:(({1}))", ac_func5, indata3));
                ac_func5++;
            }
            
        }

        /// <summary>
        /// Read and Write from other PC.
        /// </summary>
        static void func4()
        {
            string _directoryPath = "\\\\PINTO-PC";
            string _directoryPathFile = "\\\\PINTO-PC\\Users\\Pinto\\image2.jpg";
            string _userName = "Pinto";
            string _password = "7TH1TTF2";
            string localPath = "C:\\Dev\\image2.jpg";

            string src_path = "C:\\Users\\geral_000\\Dropbox\\BInteractive\\realmario.JPG";
            string dst_path = "\\\\PINTO-PC\\Users\\Pinto\\realmario.JPG";

            Console.WriteLine("Disconnect in case we are currently connected with our credentials.");
            string result_disconnect = NetworkShare.DisconnectFromShare(@_directoryPathFile, true); //Disconnect in case we are currently connected with our credentials;
            Console.WriteLine("Result:" + result_disconnect);

            Console.WriteLine("Connect with the new credentials.");
            string result_connect = NetworkShare.ConnectToShare(@_directoryPathFile, _userName, _password); //Connect with the new credentials
            Console.WriteLine("Result:" + result_connect);

            //if (!Directory.Exists(@"\\server-a\DBFiles\"))
            //    Directory.CreateDirectory(@"\\server-a\DBFiles\"+Test);

            Console.WriteLine("Copy file from Server to PC.");
            File.Copy(_directoryPathFile, localPath);
            Console.WriteLine("Copy ended.");

            Console.WriteLine("Copy file from PC to Server.");
            File.Copy(src_path, dst_path);
            Console.WriteLine("Copy ended.");

            Console.WriteLine("Disconnect from server.");
            string result_disconnect2 = NetworkShare.DisconnectFromShare(@_directoryPathFile, false); //Disconnect from the server.
            Console.WriteLine("Result:" + result_disconnect2);

            System.Console.Read();
        }

        static void func3()
        {
            //string _directoryPath = "//PINTO-PC/Users/Pinto/Doc1.docx";
            string _directoryPath = "\\\\PINTO-PC";
            string _directoryPathFile = "\\\\PINTO-PC\\Users\\Pinto\\Doc1.docx";
            string _userName = "Pinto";
            string _password = "7TH1TTF2";
            string localPath = "C:\\Dev\\Doc1.docx";

            using (new NetworkConnection(@_directoryPath, new NetworkCredential(_userName, _password)))
            {
                File.Copy(_directoryPathFile, localPath);
            }
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
            //string str = "0101";

            //System.Console.WriteLine(str[0]);
            //System.Console.WriteLine(str[1]);
            //System.Console.WriteLine(str[2]);
            //System.Console.WriteLine(str[3]);

            //System.Console.WriteLine(System.String.Format("{0}{1}{2}{3}", str[0], str[1], str[2], str[3]));

            //Console.WriteLine(12312.523.ToString("0."));

            if(File.Exists("file.txt"))
                File.Delete("file.txt");

            for (int i = 1; i <= 216; i += 8)
            {
                string order =  (i + 0) + ", " + (i + 1) + ", " + (i + 4) + ", " + (i + 5) + ", " + 
                                (i + 6) + ", " + (i + 7) + ", " + (i + 2) + ", " + (i + 3) + ", ";
                
                File.AppendAllText("file.txt", order);
                Console.Write(order);
            }


                System.Console.Read();
        }
    }
}