using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Windows.Forms;
using System.IO;
using System.Collections;
using System.Diagnostics;
using System.Drawing.Printing;


namespace CS2010_Exp
{
    class Program
    {
        static void Main(string[] args)
        {
            print_image();
        }

        static void load_image(String path)
        {

        }

        static void print_pdf()
        {
            ProcessStartInfo info = new ProcessStartInfo();
            info.Verb = "print";
            info.FileName = @"c:\image.jpg";
            info.CreateNoWindow = true;
            info.WindowStyle = ProcessWindowStyle.Hidden;

            Process p = new Process();
            p.StartInfo = info;
            p.Start();

            p.WaitForInputIdle();
            System.Threading.Thread.Sleep(15000);
            if (false == p.CloseMainWindow())
                p.Kill();
        }

        public static Image resizeImage(Image imgToResize, Size size)
        {
            return (Image)(new Bitmap(imgToResize, size));
        }

        static private void printDoc_Print(object sender, PrintPageEventArgs ev)
        {
            try
            {
                Image img = Image.FromFile(@"C:\imageV.jpg");

                float max_width = 775;
                float max_height = 1105;

                float ratio_img = (float)img.Width / (float)img.Height;
                float ratio_page = max_width / max_height;

                float w = 0, h = 0;
                float init_x = 10, init_y = 10;

                if (ratio_img > ratio_page)
                {
                    w = max_width;
                    h = img.Height * max_width / img.Width;

                    float diff = max_height - h;
                    init_y = diff / 2.0f;
                }
                else{
                    h = max_height;
                    w = img.Width * max_height / img.Height;

                    float diff = max_width - w;
                    init_x = diff / 2.0f;
                }

                Image final = resizeImage(img, new Size((int)w, (int)h));

                PointF pf = new PointF(init_x, init_y);
                ev.Graphics.DrawImage(final, pf);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }

        static void print_image()
        {
            try
            {
                PrintDocument printDoc = new PrintDocument();
                printDoc.PrintPage += new PrintPageEventHandler(printDoc_Print);
                printDoc.Print();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }
    }

    //public partial class Form1 : Form
    //{
    //    ImageList list = new ImageList();

    //    public Form1()
    //    {
    //        InitializeComponent();
    //        GetImages();
    //    }

    //    private void GetImages()
    //    {
    //        string path = @"C:\1";
    //        string[] filter = { ".bmp", ".jpg", ".jpeg", ".png" };
    //        DirectoryInfo directoryInfo = new DirectoryInfo(path);
    //        FileInfo[] fileInfo = directoryInfo.GetFiles();
    //        ArrayList arrayList = new ArrayList();

    //        foreach (FileInfo fi in fileInfo)
    //            foreach (string s in filter)
    //                if (s == fi.Extension)
    //                    arrayList.Add(fi.FullName);

    //        //adding files to image list:
    //        for (int i = 0; i < arrayList.Count; i++)
    //        {
    //            Image img = Image.FromFile(arrayList[i].ToString());
    //            list.Images.Add(img);
    //        }
    //    }
    //}
}
