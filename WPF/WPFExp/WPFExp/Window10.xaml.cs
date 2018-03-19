using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace WPFExp
{
    /// <summary>
    /// Interaction logic for Window10.xaml
    /// </summary>
    public partial class Window10 : Window
    {
        StackPanel myStackPanel;
        ScrollViewer myScrollViewerStack;

        public static string imagePath = "C:\\Dev\\Data\\Thumbnails\\";
        public List<string> allDirectories;
        public List<string> allFiles;

        public static Image[]       stackImages;
        public static BitmapImage[] myBitMaps;
        public static bool[] myBitMaps_flags;
        public static Mutex[] semaforos;
        public static BitmapImage   bitMapXX;

        public Window10()
        {
            InitializeComponent();

            //MainGrid.

            myScrollViewerStack = new ScrollViewer();
            myScrollViewerStack.VerticalScrollBarVisibility = ScrollBarVisibility.Disabled;
            myScrollViewerStack.HorizontalScrollBarVisibility = ScrollBarVisibility.Auto;
            myScrollViewerStack.ScrollChanged += myScrollViewerStack_ScrollChanged;


            myStackPanel = new StackPanel();
            myStackPanel.Orientation = Orientation.Horizontal;
            VirtualizingStackPanel.SetIsVirtualizing(myStackPanel, true);

            myScrollViewerStack.Content = myStackPanel;

            Grid.SetRow(myScrollViewerStack, 1);
            MainGrid.Children.Add(myScrollViewerStack);

            Uri url_imagem = new Uri("C:\\Dev\\Data\\Images\\XX.png");
            bitMapXX = new BitmapImage();
            bitMapXX.BeginInit();
            bitMapXX.CacheOption = BitmapCacheOption.None;
            bitMapXX.UriSource = url_imagem;
            bitMapXX.EndInit();

            allDirectories = GetAllPaths(imagePath);

            allFiles = new List<string>();

            for (int i = 0; i < allDirectories.Count; ++i)
            {
                //allFiles.AddRange(Directory.GetFiles(allDirectories[i]));
                string[] files = Directory.GetFiles(allDirectories[i]);

                for (int j = 0; j < files.Length; ++j)
                {
                    allFiles.Add(files[j]);
                }
            }

            stackImages = new Image[allFiles.Count];
            myBitMaps = new BitmapImage[allFiles.Count];
            myBitMaps_flags = new bool[allFiles.Count];
            semaforos = new Mutex[allFiles.Count];

            int counter = 0;
            for (int i = 0; i < allFiles.Count; ++i)
            {
                stackImages[i] = new Image();
                semaforos[i] = new Mutex(false);
                //imgStack.Source = MyData.bitMaps[counter];
                myBitMaps[i] = bitMapXX;
                stackImages[i].Source = myBitMaps[i];
                stackImages[i].Width = 400;
                stackImages[i].Stretch = Stretch.Uniform;

                myStackPanel.Children.Add(stackImages[i]);

                counter++;
            }

            //LoadImageAsync();
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

        public delegate void LoadFileCallback(string file_name, int idx);

        void myScrollViewerStack_ScrollChanged(object sender, ScrollChangedEventArgs e)
        {
            double start_idx_double = ((ScrollViewer)sender).HorizontalOffset / 400.0f;
            double end_idx_double = start_idx_double + ((ScrollViewer)sender).ActualWidth/ 400.0f;

            int margin = 25;

            int start_idx = (int)Math.Floor(start_idx_double);
            start_idx = ((start_idx - margin) >= 0) ? start_idx - margin : 0;
            int end_idx = (int)Math.Floor(end_idx_double);
            end_idx = ((end_idx + margin) < allFiles.Count) ? end_idx + margin : allFiles.Count;

            for (int i = 0; i < allFiles.Count; ++i)
            {
                if(i >= start_idx && i < end_idx){
                    if (!myBitMaps_flags[i])
                    {
                        string file = allFiles[i];

                        if (file.EndsWith(".jpg") ||
                            file.EndsWith(".png") ||
                            file.EndsWith(".jpeg") ||
                            file.EndsWith(".bmp"))
                        {

                            //LoadImageAsync(file, i);
                            //Thread oThread = new Thread(() => LoadImageAsync(file, i));
                            //oThread.Start();

                            //if(i == 0){
                            //    Thread oThread = new Thread(() => LoadFile_threaded(file, i));
                            //    oThread.Start();
                            //    //Thread oThread = new Thread(() => LoadImageAsync(file, i));
                            //    //oThread.Start();

                            //}

                            Uri url_imagem = new Uri(file);
                            myBitMaps[i] = new BitmapImage();
                            myBitMaps[i].BeginInit();
                            myBitMaps[i].CacheOption = BitmapCacheOption.None;
                            myBitMaps[i].UriSource = url_imagem;
                            myBitMaps[i].EndInit();

                            stackImages[i].Source = myBitMaps[i];


                            //myScrollViewerStack.Dispatcher.Invoke(
                            //    new LoadFileCallback(LoadFile),
                            //    new object[] { file, i});


                            //DispatcherFrame frame = new DispatcherFrame();
                            //Dispatcher.CurrentDispatcher.BeginInvoke(DispatcherPriority.Background,
                            //    new LoadFileCallback(LoadFile),
                            //    new object[] { file, i});
                            //Dispatcher.PushFrame(frame);


                            myBitMaps_flags[i] = true;
                        }
                    }
                }
                else
                {
                    if (myBitMaps_flags[i])
                    {
                        
                        if (semaforos[i].WaitOne())
                        {
                            myBitMaps[i] = bitMapXX;

                            stackImages[i].Source = myBitMaps[i];

                            semaforos[i].ReleaseMutex();
                        }

                        myBitMaps_flags[i] = false;
                    }
                }

                //if (semaforos[i].WaitOne())
                //{
                //    stackImages[i].Source = myBitMaps[i];

                //    stackImages[i].UpdateLayout();

                //    semaforos[i].ReleaseMutex();
                //}
            }

            GC.Collect();

            Console.WriteLine("Start (" + start_idx + ") - End(" + end_idx + ")");
        }

        public delegate void UpdateAllCallback();
        public void UpdateAll()
        {
            stackImages[0].Source = myBitMaps[0];

            //for (int i = 0; i < 20; ++i)
            //{
            //    stackImages[i].Source = myBitMaps[i];

            //    stackImages[i].UpdateLayout();
            //}
        }

        public void LoadFile(string file_name, int idx)
        {
            //Thread oThread = new Thread(() => LoadFile_threaded(file_name, idx));
            //oThread.Start();

        }

        public void LoadFile_threaded(string file_name, int idx)
        {
            Console.WriteLine("Loading " + idx + " : " + file_name);
            idx = 0;
            if (idx >= 0 && idx < 20)
            {
                //if (semaforos[idx].WaitOne())
                //{
                    Uri url_imagem = new Uri(file_name);
                    myBitMaps[idx] = new BitmapImage();
                    myBitMaps[idx].BeginInit();
                    myBitMaps[idx].CacheOption = BitmapCacheOption.None;
                    myBitMaps[idx].UriSource = url_imagem;
                    myBitMaps[idx].EndInit();
                    myBitMaps[idx].Freeze();

                
                    //stackImages[idx].Source = myBitMaps[idx];

                    //myStackPanel.Children[idx] = new Image();
                    //((Image)myStackPanel.Children[idx]).Source = myBitMaps[idx];
                    //((Image)myStackPanel.Children[idx]).Height = 400;
                    //((Image)myStackPanel.Children[idx]).Stretch = Stretch.Uniform;

                    Console.WriteLine("Loaded " + idx + " : " + file_name);


                    
                    //semaforos[idx].ReleaseMutex();

                    //myScrollViewerStack.Dispatcher.Invoke(
                    //    new UpdateAllCallback(UpdateAll),
                    //    new object[] { });

                //}
            }

            Application.Current.Dispatcher.BeginInvoke(DispatcherPriority.Send, new Action(() => UpdateAll()));            

            
        }

        private async void OnClick(object sender, RoutedEventArgs e)
        {
            //Thread oThread = new Thread(() => LoadFile_threaded("C:\\Dev\\Data\\Images\\pplware\\4k\\00.jpg", 0));
            //oThread.Start();
            
            //var progress = new Progress<BitmapImage>(img => stackImages[0].Source = img);
            //await Task.Factory.StartNew(() => LoadFile_threaded("C:\\Dev\\Data\\Images\\pplware\\4k\\00.jpg", 0, progress),
            //                            TaskCreationOptions.LongRunning);


            //var progress = new Progress<Image>(s => label.Text = s);
            //stackImages[0].Source = myBitMaps[0];

        }

        public void LoadImageAsync(string file_name, int idx = 0)
        {
            //InitializeComponent();
            BackgroundWorker worker = new BackgroundWorker();
            worker.DoWork += (sender, e) =>
            {
                BitmapImage bitmap = new BitmapImage(new Uri(e.Argument.ToString(), UriKind.RelativeOrAbsolute));
                bitmap.Freeze(); // Freeze this content so you can pass it safely across thread boundaries.
                e.Result = bitmap;
            };

            worker.RunWorkerCompleted += (sender, e) =>
            {
                BitmapImage bitmap = e.Result as BitmapImage;
                if (bitmap != null)
                {
                    stackImages[0].Source = bitmap;
                }
            };

            //btn.Click += delegate
            //{
                worker.RunWorkerAsync(@file_name);
            //};
        }
    }
}
