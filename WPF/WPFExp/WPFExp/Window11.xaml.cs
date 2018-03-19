using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace WPFExp
{
    /// <summary>
    /// Interaction logic for Window11.xaml
    /// </summary>
    public partial class Window11 : Window
    {
        public static string imagePath = "C:\\Dev\\Data\\Images\\";
        public static string thumbnailsPath = "C:\\Dev\\Data\\Thumbnails\\";
        public static int thumbnailWidth = 400;
        public static int thumbnailHeight = 300;
        public List<string> allDirectories;
        public List<string> allFiles;
        public List<string> allThumbnailFiles;

        public Window11()
        {
            InitializeComponent();

            allDirectories = GetAllPaths(imagePath);

            allFiles = new List<string>();

            for (int i = 0; i < allDirectories.Count; ++i)
            {
                allFiles.AddRange(Directory.GetFiles(allDirectories[i]));
            }

            allThumbnailFiles = new List<string>();

            for (int i = 0; i < allFiles.Count; ++i)
            {
                allThumbnailFiles.Add(allFiles[i].Replace(imagePath, thumbnailsPath));
            }

            GenerateAllThumbnails();

            Console.WriteLine();
        }

        public List<string> GetAllPaths(string path)
        {
            List<string> directoriesCollection = new List<string>();

            directoriesCollection.Add(path);

            foreach (string directory in Directory.EnumerateDirectories(path))
            {
                directoriesCollection.AddRange(GetAllPaths(directory));
            }

            return directoriesCollection;
        }

        public void GenerateAllThumbnails()
        {
            for (int i = 0; i < allThumbnailFiles.Count; ++i)
            {
                if (allThumbnailFiles[i].EndsWith(".jpg") ||
                    allThumbnailFiles[i].EndsWith(".png") ||
                    allThumbnailFiles[i].EndsWith(".jpeg")||
                    allThumbnailFiles[i].EndsWith(".bmp"))
                {
                    if (!File.Exists(allThumbnailFiles[i])) //File does not exist
                    {
                        if (!Directory.Exists(allThumbnailFiles[i].Substring(0, allThumbnailFiles[i].LastIndexOf("\\"))))
                        {
                            Directory.CreateDirectory(allThumbnailFiles[i].Substring(0, allThumbnailFiles[i].LastIndexOf("\\")));
                        }

                        CreateThumbnail(allFiles[i], allThumbnailFiles[i]);
                    }
                }
            }

        }

        private static void CreateThumbnail(string orig, string thumbnail)
        {         
            try
            {
                Uri url_imagem = new Uri(orig);
                //Decoder para receber o tamanho da imagem e para receber o formato da imagem (extensão)
                BitmapDecoder BmpDecoder = BitmapDecoder.Create(url_imagem, BitmapCreateOptions.DelayCreation, BitmapCacheOption.None);
                int imageHeight = BmpDecoder.Frames[0].PixelHeight;
                int imageWidth = BmpDecoder.Frames[0].PixelWidth;

                if (imageHeight < thumbnailHeight && imageWidth < thumbnailWidth) // SMALL IMAGE
                    return;

                Guid ExtensionFormat = BmpDecoder.CodecInfo.ContainerFormat;

                //Criar o Thumbnail da Imagem

                BitmapImage BitMap = new BitmapImage();
                BitMap.BeginInit();
                BitMap.UriSource = url_imagem;

                if (imageHeight > thumbnailHeight)
                {
                    double ratio = (double)imageWidth / (double)imageHeight;
                    imageHeight = thumbnailHeight;
                    imageWidth = Convert.ToInt32(imageHeight * ratio);
                }
                if (imageWidth > thumbnailWidth)
                {
                    double ratio = (double)imageHeight / (double)imageWidth;
                    imageWidth = thumbnailWidth;
                    imageHeight = Convert.ToInt32(imageWidth * ratio);
                }
                BitMap.DecodePixelWidth = imageWidth;
                BitMap.DecodePixelHeight = imageHeight;
                BitMap.EndInit();
                BitMap.Freeze();
                PixelFormat PixelFormat = BitMap.Format;

                double dpi = 96;

                int stride = imageWidth * 4; // 4 bytes per pixel
                byte[] pixelData = new byte[stride * imageHeight];
                BitMap.CopyPixels(pixelData, stride, 0);

                BitmapSource bmpSource = BitmapSource.Create(imageWidth, imageHeight, dpi, dpi, PixelFormat, null, pixelData, stride);

                BitmapEncoder BmpEncoder = BitmapEncoder.Create(ExtensionFormat);
                BmpEncoder.Frames.Add(BitmapFrame.Create(bmpSource));

                FileStream thumbnailStream = new FileStream(thumbnail, FileMode.Create);
                BmpEncoder.Save(thumbnailStream);
                thumbnailStream.Close();
            }
            catch (Exception ex)
            {
                Console.WriteLine("CreateThumbnail Exception - " + ex.ToString());                
            }
        }
    }
}
