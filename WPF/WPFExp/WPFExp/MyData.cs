using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace WPFExp
{
    class MyData
    {
        public static string                    path = "C:\\Dev\\Data\\Images\\Numbers\\";
        public static IEnumerable<string>       files;
        public static BitmapImage[]             bitMaps;
        public static Image[]                   images;

        public static BitmapImage               bitMapOver;
        public static Image                     imageOver;


        public static BitmapImage               bitMapLabelSearch;

        public static void LoadImages()
        {
            files = Directory.EnumerateFiles(path);

            long milliseconds_start = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

            images  = new Image[files.Count()];
            bitMaps = new BitmapImage[files.Count()];

            for (int i = 0; i < files.Count(); ++i)
            {
                if (files.ElementAt<string>(i).EndsWith(".png") || files.ElementAt<string>(i).EndsWith(".jpg"))
                {
                    //images[i] = new Image
                    //{
                    //    Source = new BitmapImage(new Uri(files.ElementAt<string>(i), UriKind.Absolute))
                    //};

                    //images[i].Stretch = Stretch.Uniform;

                    Uri url_imagem = new Uri(files.ElementAt<string>(i));
                    bitMaps[i] = new BitmapImage();
                    bitMaps[i].BeginInit();
                    bitMaps[i].CacheOption = BitmapCacheOption.OnDemand;
                    bitMaps[i].UriSource = url_imagem;
                    bitMaps[i].EndInit();

                    //images[i] = new Image();
                    //images[i].Source = bitMaps[i];
                    //images[i].Stretch = Stretch.Uniform;
                }
            }

            long milliseconds_end = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

            Console.WriteLine("Duration: " + (milliseconds_start - milliseconds_end) + "(ms)");


            
            bitMapOver = new BitmapImage();
            bitMapOver.BeginInit();
            bitMapOver.CacheOption = BitmapCacheOption.OnLoad;
            bitMapOver.UriSource = new Uri("C:\\Dev\\Data\\Images\\colors.jpg", UriKind.Absolute);
            bitMapOver.EndInit();


            bitMapLabelSearch = new BitmapImage();
            bitMapLabelSearch.BeginInit();
            bitMapLabelSearch.CacheOption = BitmapCacheOption.OnLoad;
            bitMapLabelSearch.UriSource = new Uri("./Textures/LabelSearch.png", UriKind.Relative);
            //bitMapLabelSearch.UriSource = new Uri("Textures/LabelSearch.png");
            bitMapLabelSearch.EndInit();


            imageOver = new Image();
            imageOver.Source = bitMapOver;
            imageOver.Stretch = Stretch.Uniform;
        }
    }
}
