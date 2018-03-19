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
using System.Threading;
using System.Windows.Media.Animation;  

namespace WPFExp
{


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public static RoutedCommand MyCommand = new RoutedCommand();

        bool            splashScreenFlag = false;
        SplashScreen    splashScreen = null;

        Window01        window01 = null;
        Window02        window02 = null;
        Window03        window03 = null;
        Window04        window04 = null;
        Window05        window05 = null;
        Window06        window06 = null;
        Window07        window07 = null;
        Window08        window08 = null;
        
        Window10        window10 = null;
        Window11        window11 = null;

        public MainWindow()
        {
            InitializeComponent();

            MyData.LoadImages();
            //MyStyles.LoadStyles();

            MyCommand.InputGestures.Add(new KeyGesture(Key.D1, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D2, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D3, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D4, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D5, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D6, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D7, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D8, ModifierKeys.Control));
            MyCommand.InputGestures.Add(new KeyGesture(Key.D9, ModifierKeys.Control));
        }

      

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            switch ((string)((Button)sender).Tag)
            {
                case "BT01":
                    if (splashScreenFlag)
                    {
                        splashScreen.Close(new TimeSpan(0, 0, 2));
                        splashScreenFlag = false;
                    }
                    else
                    {
                        splashScreen = new SplashScreen("SplashScreen1.png");
                        
                        splashScreen.Show(false);
                        splashScreenFlag = true;
                    }                   
                    
                    break;
                case "BT02":
                    window01 = new Window01();

                    //window01.WindowState = System.Windows.WindowState.Maximized;
                    //window01.WindowStyle = System.Windows.WindowStyle.None;

                    window01.Show();
                    break;
                case "BT03":
                    long milliseconds_start = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;

                    if (window02 == null)
                    {
                        window02 = new Window02();

                        window02.WindowState = System.Windows.WindowState.Maximized;
                        window02.WindowStyle = System.Windows.WindowStyle.None;

                        window02.Show();

                        window02.PopulateGrid();
                    }
                    else
                    {
                        window02.fadeInAll();
                        window02.Show();
                    }

                    long milliseconds_end = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                    Console.WriteLine("Window 02: " + (milliseconds_start - milliseconds_end) + "(ms)");

                    break;
                case "BT04":
                    window03 = new Window03();
                    window03.Show();
                    
                    break;
                case "BT05":
                    window04 = new Window04();
                    window04.Show();
                    break;
                case "BT06":
                    window05 = new Window05();
                    window05.Show();
                    break;
                case "BT07":
                    window06 = new Window06();
                    window06.Show();
                    break;
                case "BT08":
                    window07 = new Window07();
                    window07.Show();
                    break;
                case "BT09":
                    window08 = new Window08();
                    window08.Show();
                    break;
                case "BT10":
                    window10 = new Window10();
                    window10.Show();
                    break;
                case "BT11":
                    window11 = new Window11();
                    window11.Show();
                    break;
                case "BT12":

                    break;
                default: break;
            }
        }

        private void MyCommandExecuted(object sender, ExecutedRoutedEventArgs e)
        {
            //Console.WriteLine("Key: " + ((RoutedCommand)e.Command).Name);
            Console.WriteLine("Key");
        }
    }
}
