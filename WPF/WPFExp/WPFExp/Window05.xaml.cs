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
using System.Windows.Shapes;

namespace WPFExp
{
    /// <summary>
    /// Interaction logic for Window05.xaml
    /// </summary>
    public partial class Window05 : Window
    {
        Grid leftGrid;

        StackPanel rightStackPanel;
        ScrollViewer rightScrollViewer;

        public Window05()
        {
            InitializeComponent();
            
            //this.Width = 350;
            //this.MinWidth = 350;

            //this.Height = 350;
            //this.MinHeight = 350;

            this.WindowState = System.Windows.WindowState.Maximized;
            this.WindowStyle = System.Windows.WindowStyle.None;
            this.AllowsTransparency = true;


            PrepareGUI();

            CreateLeftGrid();

            PopulateLeftHorizontalSlideShow1();

            CreateRightVerticalSlideShow();
            PopulateRightVerticalSlideShow();
        }

        private void PrepareGUI()
        {
            //this.WindowStyle = WindowStyle.None;
            //this.AllowsTransparency = true;

            //MainGrid.Width = 400;
            //MainGrid.HorizontalAlignment = HorizontalAlignment.Left;
            //MainGrid.VerticalAlignment = VerticalAlignment.Top;
            MainGrid.ShowGridLines = true;
            MainGrid.Background = new SolidColorBrush(Colors.Black);


            for (int i = 0; i < 2; ++i)
            {
                ColumnDefinition col = new ColumnDefinition();
                ////if (i == 1)
                //{
                //    col.Width = new GridLength(.5, GridUnitType.Star);
                //}
                MainGrid.ColumnDefinitions.Add(col);
            }

            MainGrid.ColumnDefinitions.ElementAt(0).Width = new GridLength(.6667, GridUnitType.Star);
            //MainGrid.ColumnDefinitions.ElementAt(0).MinWidth = 200;
            MainGrid.ColumnDefinitions.ElementAt(1).Width = new GridLength(.3333, GridUnitType.Star);
            //MainGrid.ColumnDefinitions.ElementAt(1).MinWidth = 100;

        }

        private void CreateLeftGrid()
        {
            leftGrid = new Grid();
            leftGrid.ShowGridLines = true;
            leftGrid.Background = new SolidColorBrush(Colors.DarkGray);
            for (int i = 0; i < 3; ++i)
            {
                RowDefinition row = new RowDefinition();
                leftGrid.RowDefinitions.Add(row);
            }


            //TOP SLIDER
            ScrollViewer scrollTop = new ScrollViewer();
            scrollTop.VerticalScrollBarVisibility = ScrollBarVisibility.Disabled;
            scrollTop.HorizontalScrollBarVisibility = ScrollBarVisibility.Hidden;

            scrollTop.PanningMode = PanningMode.HorizontalOnly;
            scrollTop.PanningDeceleration = 0.001;
            scrollTop.ManipulationBoundaryFeedback += HandlerManipulationBoundaryFeedback;

            StackPanel stackTop = new StackPanel();
            stackTop.Orientation = Orientation.Horizontal;
            //stackTop.MouseDown += stackTop_MouseDown;

            scrollTop.Content = stackTop;

            Grid.SetRow(scrollTop, 0);
            leftGrid.Children.Add(scrollTop);



            //MIDDLE SLIDER
            ScrollViewer scrollMiddle= new ScrollViewer();
            scrollMiddle.VerticalScrollBarVisibility = ScrollBarVisibility.Disabled;
            scrollMiddle.HorizontalScrollBarVisibility = ScrollBarVisibility.Auto;


            StackPanel stackMiddle= new StackPanel();
            stackMiddle.Orientation = Orientation.Horizontal;

            scrollMiddle.Content = stackMiddle;

            Grid.SetRow(scrollMiddle, 1);
            leftGrid.Children.Add(scrollMiddle);




            //DOWN SLIDER
            ScrollViewer scrollDown= new ScrollViewer();
            scrollDown.VerticalScrollBarVisibility = ScrollBarVisibility.Disabled;
            scrollDown.HorizontalScrollBarVisibility = ScrollBarVisibility.Auto;


            StackPanel stackDown= new StackPanel();
            stackDown.Orientation = Orientation.Horizontal;

            scrollDown.Content = stackDown;

            Grid.SetRow(scrollDown, 2);
            leftGrid.Children.Add(scrollDown);

           // UIElement elem= leftGrid.Children.Cast<UIElement>().First(e => Grid.GetRow(e) == 0 && Grid.GetColumn(e) == 2);
            //elem.MouseDown += stackTop_MouseDown;
            //leftGrid.MouseDown += stackTop_MouseDown;

            UIElement elem= leftGrid.Children.Cast<UIElement>().First(e => Grid.GetRow(e) == 2 && Grid.GetColumn(e) == 0);

            elem.TouchDown += leftGrid_TouchDown;
            elem.MouseDown += stackTop_MouseDown;
            





            Grid.SetRow(leftGrid, 0);
            Grid.SetColumn(leftGrid, 0);
            MainGrid.Children.Add(leftGrid);




            //MainGrid.Children.Cast<UIElement>().First(e => Grid.GetRow(e) == 1 && Grid.GetColumn(e) == 1);
        }

        void leftGrid_TouchDown(object sender, TouchEventArgs e)
        {
//            e.Device.ToString();
            Console.WriteLine("Touch Down: " + e.GetTouchPoint(this).Position.X + "," + e.GetTouchPoint(this).Position.Y);
        }

        void stackTop_MouseDown(object sender, MouseButtonEventArgs e)
        {
            //Mouse.GetPosition()

            //((StackPanel)sender).vis

            //MessageBox.Show("Mouse Down");
            Console.WriteLine("Mouse Down: " + e.GetPosition(this).X + "," + e.GetPosition(this).Y);
            //Console.WriteLine("Touch");
        }

        private void CreateRightVerticalSlideShow()
        {
            rightScrollViewer= new ScrollViewer();
            rightScrollViewer.VerticalScrollBarVisibility = ScrollBarVisibility.Hidden;
            rightScrollViewer.HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled;

            rightScrollViewer.PanningMode = PanningMode.VerticalOnly;
            rightScrollViewer.PanningDeceleration = 0.001;

            rightStackPanel = new StackPanel();
            rightStackPanel.Orientation = Orientation.Vertical;

            rightScrollViewer.ManipulationBoundaryFeedback += HandlerManipulationBoundaryFeedback;


            rightScrollViewer.Content = rightStackPanel;

            Grid.SetColumn(rightScrollViewer, 1);
            MainGrid.Children.Add(rightScrollViewer);
        }

        void HandlerManipulationBoundaryFeedback(object sender, ManipulationBoundaryFeedbackEventArgs e)
        {
            e.Handled = true;
        }


        void PopulateRightVerticalSlideShow()
        {
            int counter = 0;

            for (int i = 0; i < MyData.files.Count(); ++i)
            {
                Grid imageGrid = new Grid();
                imageGrid.ShowGridLines = true;
                imageGrid.Background = new SolidColorBrush(Colors.LightGray);

                for (int c = 0; c < 2; ++c)
                {
                    ColumnDefinition col = new ColumnDefinition();
                    imageGrid.ColumnDefinitions.Add(col);
                }

                Image imgStack = new Image();
                imgStack.Source = MyData.bitMaps[counter++];
                imgStack.Stretch = Stretch.Uniform;

                Grid.SetColumn(imgStack, 0);
                imageGrid.Children.Add(imgStack);


                Image imgOver= new Image();
                imgOver.Source = MyData.bitMapOver;
                imgOver.Stretch = Stretch.Uniform;

                Grid.SetColumn(imgOver, 1);
                imageGrid.Children.Add(imgOver);

                imageGrid.Margin = new Thickness(0, 0, 0, 10);


                rightStackPanel.Children.Add(imageGrid);
            }
        }

        void PopulateLeftHorizontalSlideShow1(){
            int counter = 0;

            for (int i = 0; i < MyData.files.Count(); ++i)
            {
                Image imgStack = new Image();
                imgStack.Source = MyData.bitMaps[counter++];
                imgStack.Stretch = Stretch.Uniform;

                UIElement sliderTop = leftGrid.Children.Cast<UIElement>().First(e => Grid.GetRow(e) == 0 && Grid.GetColumn(e) == 0);

                StackPanel stackPanel = (StackPanel)(((ScrollViewer)sliderTop).Content);

                stackPanel.Children.Add(imgStack);
            }
        }
    }
}


/*
             for (int i = 0; i < 2; ++i)
            {
                ColumnDefinition col = new ColumnDefinition();
                MainGrid.ColumnDefinitions.Add(col);
            }
            for (int i = 0; i < 2; ++i)
            {
                RowDefinition row = new RowDefinition();
                MainGrid.RowDefinitions.Add(row);
            }

            int counter = 0;
            for (int r = 0; r < MainGrid.RowDefinitions.Count; ++r)
            {
                for (int c = 0; c < MainGrid.ColumnDefinitions.Count; ++c)
                {
                    Image img = new Image();
                    img.Source = MyData.bitMaps[counter];
                    img.Stretch = Stretch.Uniform;


                    Grid.SetRow(img, r);
                    Grid.SetColumn(img, c);
                    MainGrid.Children.Add(img);

                    counter++;

                    if (counter >= MyData.images.Length)
                        return;
                }
            }
 */