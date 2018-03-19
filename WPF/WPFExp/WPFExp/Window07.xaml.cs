using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
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
    /// Interaction logic for Window07.xaml
    /// </summary>
    public partial class Window07 : Window
    {
        Grid topGrid;

        ScrollViewer bottomScroll;
        StackPanel bottomStack;


        TextBox text;

        public Window07()
        {
            InitializeComponent();

            this.WindowState = System.Windows.WindowState.Maximized;
            //this.WindowStyle = System.Windows.WindowStyle.None;
            //this.AllowsTransparency = true;

            PrepareGUI();

            PopulateTopGrid();
        }

        private void PrepareGUI()
        {
            for (int i = 0; i < 2; ++i)
            {
                RowDefinition row = new RowDefinition();
                MainGrid.RowDefinitions.Add(row);
            }

            MainGrid.RowDefinitions.ElementAt(0).Height = new GridLength(.05, GridUnitType.Star);
            MainGrid.RowDefinitions.ElementAt(1).Height = new GridLength(.95, GridUnitType.Star);



            //TOP
            topGrid = new Grid();
            topGrid.ShowGridLines = true;
            
            Grid.SetRow(topGrid, 0);
            MainGrid.Children.Add(topGrid);


            for (int i = 0; i < 4; ++i)
            {
                ColumnDefinition col = new ColumnDefinition();
                topGrid.ColumnDefinitions.Add(col);
            }

            //SEARCH LABEL
            Image img = new Image();
            img.Source = MyData.bitMapLabelSearch;
            img.Stretch = Stretch.Fill;

            Grid.SetColumn(img, 0);
            topGrid.Children.Add(img);

            //COMBOBOX 
            ComboBox combo = new ComboBox();
            combo.Items.Add("All Items");
            combo.Items.Add("Item1");
            combo.Items.Add("Item2");
            combo.Items.Add("Item3");
            combo.Items.Add("Item4");
            combo.Items.Add("Item5");
            combo.SelectedIndex = 0;

            Grid.SetColumn(combo, 1);
            topGrid.Children.Add(combo);


            //TEXTBOX
            text = new TextBox();
            text.Text = "Insert Text";
            text.TouchDown += text_TouchDown;
            text.GotFocus += text_GotFocus;
            text.LostFocus += text_LostFocus;

            Grid.SetColumn(text, 2);
            topGrid.Children.Add(text);

            //BUTTON
            Button bt = new Button();
            bt.Content = "Ok";
            bt.Click += bt_Click;

            Grid.SetColumn(bt, 3);
            topGrid.Children.Add(bt);

            bt.Width = bt.Height;


            //imageGrid.RowDefinitions.ElementAt(0).Height = new GridLength(.70, GridUnitType.Star);
            //imageGrid.RowDefinitions.ElementAt(1).Height = new GridLength(.15, GridUnitType.Star);
            //imageGrid.RowDefinitions.ElementAt(2).Height = new GridLength(.15, GridUnitType.Star);


            //BOTTOM 
            bottomScroll = new ScrollViewer();
            bottomScroll.VerticalScrollBarVisibility = ScrollBarVisibility.Disabled;
            bottomScroll.HorizontalScrollBarVisibility = ScrollBarVisibility.Hidden;

            bottomScroll.PanningMode = PanningMode.HorizontalOnly;
            bottomScroll.PanningDeceleration = 0.001;
            bottomScroll.ManipulationBoundaryFeedback += HandlerManipulationBoundaryFeedback;

            bottomStack = new StackPanel();
            bottomStack.Orientation = Orientation.Horizontal;


            //bottomStack.MouseDown += stackTop_MouseDown;

            bottomScroll.Content = bottomStack;

            Grid.SetRow(bottomScroll, 1);
            MainGrid.Children.Add(bottomScroll);
        }


        void bt_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("Button Clicked");
        }



        [DllImport("user32.dll")]
        public static extern int FindWindow(string lpClassName, string lpWindowName);

        [DllImport("user32.dll")]
        public static extern int SendMessage(int hWnd, uint Msg, int wParam, int lParam);

        public const int WM_SYSCOMMAND = 0x0112;
        public const int SC_CLOSE = 0xF060;

        void text_LostFocus(object sender, RoutedEventArgs e)
        {
            int iHandle = FindWindow("IPTIP_Main_Window", "");
            if (iHandle > 0)
            {
                // close the window using API        
                SendMessage(iHandle, WM_SYSCOMMAND, SC_CLOSE, 0);
            }
        }

        void text_GotFocus(object sender, RoutedEventArgs e)
        {
            int iHandle = FindWindow("IPTIP_Main_Window", "");
            if (iHandle > 0)
            {
                System.Diagnostics.Process.Start(@"C:\Program Files\Common Files\Microsoft Shared\ink\TabTip.exe");
            }
        }

        void text_TouchDown(object sender, TouchEventArgs e)
        {
            int iHandle = FindWindow("IPTIP_Main_Window", "");
            if (iHandle > 0)
            {
                System.Diagnostics.Process.Start(@"C:\Program Files\Common Files\Microsoft Shared\ink\TabTip.exe");
            }
        }
        private void HandlerManipulationBoundaryFeedback(object sender, ManipulationBoundaryFeedbackEventArgs e)
        {
            e.Handled = true;
        }

        private void PopulateTopGrid()
        {
            for (int i = 0; i < MyData.files.Count(); ++i)
            {
                Image imgStack = new Image();
                imgStack.Source = MyData.bitMaps[i++];
                imgStack.Stretch = Stretch.Fill;

                bottomStack.Children.Add(imgStack);
            }
        }
    }
}
