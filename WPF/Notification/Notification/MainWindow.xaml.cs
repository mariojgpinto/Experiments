using System;
using System.Collections.Generic;
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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace Notification
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        DispatcherTimer timer;

        //private int startPosX;
        private int startPosY = -150;

        public MainWindow()
        {
            InitializeComponent();

            //myMediaElement = new MediaElement();
            //myMediaElement.

            RowDefinition myRowDef1 = new RowDefinition();
            PedoGrid.RowDefinitions.Add(myRowDef1);

            Image img = new Image
                {
                    Source = new BitmapImage(new Uri("creep2.png", UriKind.Relative))
                };

            img.Stretch = Stretch.Uniform;

            Grid.SetRow(img, 0);
            Grid.SetColumn(img, 0);

            PedoGrid.Children.Add(img);
            

            // We want our window to be the top most
            Topmost = true;

            // Pop doesn't need to be shown in task bar
            ShowInTaskbar = false;

            timer = new DispatcherTimer();
            // Specify timer interval.
            timer.Interval = new TimeSpan(0, 0, 0, 0, 18);
            // Specify timer event handler function.
            timer.Tick += new EventHandler(TimerTickShow);

            timer.Start();
        }

        bool sound = false;
        void TimerTickShow(object sender, EventArgs e)
        {
            //Lift window by 5 pixels
            startPosY += 5;
            //If window is fully visible stop the timer
            if (startPosY > 0)
            {
                startPosY = 0;

                timer.Stop();

				if (!sound) {
					MediaPlayer player = new MediaPlayer();
					//player.Open(new Uri("C:\\yeehee.wav", UriKind.Relative));

					player.Open(new Uri("C:\\Windows\\Notifications\\yeehee.wav"));
					player.Play();

					sound = true;
				}

				System.Threading.Thread.Sleep(1500);

				timer.Interval = new TimeSpan(0, 0, 0, 0, 18 );
                // Specify timer event handler function.
                timer.Tick += new EventHandler(TimerTickHide);

                timer.Start();

               
            }
            else
            {
                //var location = myTextBlock.PointToScreen(new Point(0, 0));
                //this.Left = startPosY;
                this.Top = startPosY;

            }
        }

        void TimerTickHide(object sender, EventArgs e)
        {
            //Lift window by 5 pixels
            startPosY -= 5;
            //If window is fully visible stop the timer
            if (startPosY < -Height)
            {
                timer.Stop();

                System.Threading.Thread.Sleep(500);
                this.Close();
            }
            else
            {
                this.Top = startPosY;
            }
        }
    }
}
