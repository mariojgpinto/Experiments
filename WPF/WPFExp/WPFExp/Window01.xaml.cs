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
using System.Windows.Threading;

namespace WPFExp
{
    /// <summary>
    /// Interaction logic for Window01.xaml
    /// </summary>
    public partial class Window01 : Window
    {
        Canvas imageCanvas;
    
        // Declaring a timer.
        DispatcherTimer timer;

        // counter to track images.
        int counter = 0;
        
        
        
        public Window01()
        {
            InitializeComponent();          

            ColumnDefinition col = new ColumnDefinition();
            //col.Width = GridLength.Auto;
            RowDefinition row = new RowDefinition();
            //row.Height = GridLength.Auto;

            SlideShowGrid.ColumnDefinitions.Add(col);
            SlideShowGrid.RowDefinitions.Add(row);

            //imageCanvas = new Canvas();

            //Grid.SetRow(imageCanvas, 0);
            //Grid.SetColumn(imageCanvas, 0);

            //SlideShowGrid.Children.Add(imageCanvas);

            PlaySlideShow(0);

            timer = new DispatcherTimer();
            // Specify timer interval.
            timer.Interval = new TimeSpan(0, 0, 0, 0, 50);
            // Specify timer event handler function.
            timer.Tick += new EventHandler(TimerTick);

            timer.Start();
        }

        void TimerTick(object sender, EventArgs e)
        {
            counter = (counter + 1) % MyData.images.Count();
            
            PlaySlideShow(counter);
        }

        private void PlaySlideShow(int counter)
        // Function to display image.
        {
            if (SlideShowGrid.Children.Count > 0)
                SlideShowGrid.Children.RemoveAt(0);

            Image img = new Image();
            img.Source = MyData.bitMaps[counter];
            img.Stretch = Stretch.Uniform;

            Grid.SetRow(img, 0);
            Grid.SetColumn(img, 0);

            //MyData.images[counter].;

            SlideShowGrid.Children.Add(img);
        }
    }
}
