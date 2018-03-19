using System;
using System.Collections.Generic;
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
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace WPFExp
{
    /// <summary>
    /// Interaction logic for Window02.xaml
    /// </summary>
    public partial class Window02 : Window
    {
        static bool loaded = false;
        
        public Window02()
        {
            InitializeComponent();


            long milliseconds_start = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

            CreateGrid(MyData.images.Length);

            //PopulateGridThread();
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            e.Cancel = true;
            AllPhotoGrid.Opacity = 0;
            this.Hide();
        }

        public void fadeInAll()
        {
            var fadeInAnimation = new DoubleAnimation(0.0, 1.0, new Duration(new TimeSpan(0, 0, 1)));
            AllPhotoGrid.BeginAnimation(Grid.OpacityProperty, fadeInAnimation);
        }


        void PopulateGridThread()
        {
            Thread t = new Thread(this.PopulateGrid);
            t.Start();
        }
        
        void CreateGrid(int nPhotos)
        {
            int delta_rows = 1;
            int delta_cols = 1;

            bool flag = false;
            while (nPhotos > (delta_rows * delta_cols))
            {
                if (flag)
                {
                    delta_rows++;
                }
                else
                {
                    delta_cols++;
                }
                flag = !flag;
            }

            for (int c = AllPhotoGrid.ColumnDefinitions.Count ; c < delta_cols; ++c)
            {
                ColumnDefinition col = new ColumnDefinition();
                AllPhotoGrid.ColumnDefinitions.Add(col);
            }

            for (int r = AllPhotoGrid.RowDefinitions.Count; r < delta_rows; ++r)
            {
                RowDefinition row = new RowDefinition();
                AllPhotoGrid.RowDefinitions.Add(row);
            }
        }

        public void PopulateGrid()
        {
            int counter = 0;

            Console.WriteLine("ColCounter: " + AllPhotoGrid.ColumnDefinitions.Count);
            Console.WriteLine("RowCounter: " + AllPhotoGrid.RowDefinitions.Count);

            for (int r = 0; r < AllPhotoGrid.RowDefinitions.Count; ++r)
            {
                for (int c = 0; c < AllPhotoGrid.ColumnDefinitions.Count; ++c)
                {
                    Border border = new Border();
                    border.Background = new SolidColorBrush(Colors.LightGray);
                    border.BorderThickness = new Thickness(2);
                    border.BorderBrush = new SolidColorBrush(Colors.Red);
                    border.CornerRadius = new CornerRadius(5);
                    border.Opacity = 0;
                    border.Tag = counter.ToString();
                    border.MouseEnter += Window02_MouseEnter;
                    border.MouseLeave += border_MouseLeave;


                    Image img = new Image();
                    img.Source = MyData.bitMaps[counter];
                    img.Stretch = Stretch.Uniform;

                    border.Child = img;
                                               

                    Grid.SetRow(border, r);
                    Grid.SetColumn(border, c);
                    AllPhotoGrid.Children.Add(border);

                    var fadeInAnimation = new DoubleAnimation(1d, new Duration(new TimeSpan(0, 0, 1)));
                    border.BeginAnimation(Image.OpacityProperty, fadeInAnimation);

                    DispatcherFrame frame = new DispatcherFrame();
                    Dispatcher.CurrentDispatcher.BeginInvoke(DispatcherPriority.Background,
                        new DispatcherOperationCallback(ExitFrame), frame);
                    Dispatcher.PushFrame(frame);

                    counter++;

                    //Console.WriteLine("Photo Loading at " + counter + "/" + MyData.images.Length);

                    if (counter >= MyData.images.Length)
                        return;
                }
            }
        }

        public object ExitFrame(object f)
        {
            ((DispatcherFrame)f).Continue = false;

            return null;
        }
        
        void border_MouseLeave(object sender, MouseEventArgs e)
        {
            int idx = Convert.ToInt32(((Border)sender).Tag);

            Image img = new Image();
            img.Source = MyData.bitMaps[idx];
            img.Stretch = Stretch.Uniform;

            ((Border)sender).Child = img;
        }

        void Window02_MouseEnter(object sender, MouseEventArgs e)
        {
            ((Border)sender).Child = MyData.imageOver;
        }
    }
}
