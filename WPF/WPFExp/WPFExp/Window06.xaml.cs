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
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace WPFExp
{
    /// <summary>
    /// Interaction logic for Window06.xaml
    /// </summary>
    public partial class Window06 : Window
    {
        ScrollViewer        topScroll;
        StackPanel          topStack;
        
        ScrollViewer        bottomScroll;
        StackPanel          bottomStack;



        public Window06()
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

            MainGrid.RowDefinitions.ElementAt(1).Height = new GridLength(300, GridUnitType.Pixel);

            //TOP SCROLL
            topScroll = new ScrollViewer();
            topScroll.VerticalScrollBarVisibility = ScrollBarVisibility.Disabled;
            topScroll.HorizontalScrollBarVisibility = ScrollBarVisibility.Hidden;
            topScroll.Background = Brushes.DarkGray;
            topScroll.MinHeight = 150;

            topScroll.PanningMode = PanningMode.HorizontalOnly;
            topScroll.PanningDeceleration = 0.001;
            topScroll.ManipulationBoundaryFeedback += HandlerManipulationBoundaryFeedback;

            topStack = new StackPanel();
            topStack.Orientation = Orientation.Horizontal;
            //stackTop.MouseDown += stackTop_MouseDown;

            topScroll.Content = topStack;

            Grid.SetRow(topScroll, 0);
            MainGrid.Children.Add(topScroll);




            //BOTTOM SCROLL
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

        private void HandlerManipulationBoundaryFeedback(object sender, ManipulationBoundaryFeedbackEventArgs e)
        {
            e.Handled = true;
        }

        private void PopulateTopGrid()
        {
            int counter = 0;

            for (int i = 0; i < MyData.files.Count(); ++i)
            {
                Grid imageGrid = new Grid();
                for (int j = 0; j < 3; ++j)
                {
                    RowDefinition row = new RowDefinition();
                    imageGrid.RowDefinitions.Add(row);
                }
                imageGrid.ShowGridLines = true;
                imageGrid.Tag = "" + i;

                imageGrid.RowDefinitions.ElementAt(0).Height = new GridLength(.70, GridUnitType.Star);
                imageGrid.RowDefinitions.ElementAt(1).Height = new GridLength(.15, GridUnitType.Star);
                imageGrid.RowDefinitions.ElementAt(2).Height = new GridLength(.15, GridUnitType.Star);

                Image imgStack = new Image();
                imgStack.Source = MyData.bitMaps[counter++];
                imgStack.Stretch = Stretch.Fill;

                Grid.SetRow(imgStack, 0);
                Grid.SetRowSpan(imgStack, 3);
                imageGrid.Children.Add(imgStack);



                CheckBox checkBox = new CheckBox();
                checkBox.Style = this.FindResource("myCheckboxStyle") as Style;

                checkBox.Checked += checkBox_Checked;
                checkBox.Unchecked += checkBox_Unchecked;
                checkBox.Tag = "" + i;


                Grid.SetRow(checkBox, 1);
                imageGrid.Children.Add(checkBox);
                
                topStack.Children.Add(imageGrid);

                Expander expander = new Expander();
                expander.Style = this.FindResource("myExpanderStyle") as Style;
                //expander.Background = Brushes.LightGray;
                //expander.Opacity = 0.7;
                ////expander.HorizontalAlignment = System.Windows.HorizontalAlignment.Right;
                //expander.HorizontalContentAlignment = System.Windows.HorizontalAlignment.Right;
                expander.Expanded += expander_Expanded;
                expander.Collapsed += expander_Collapsed;

                StackPanel expanderPanel = new StackPanel();

                for(int k = 0 ; k < 5 ; ++k){
                    Label text = new Label();
                    text.Content = "Label" + k;

                    expanderPanel.Children.Add(text);
                }

                expander.Content = expanderPanel;

                Grid.SetRow(expander, 7);
                imageGrid.Children.Add(expander);
            }
        }

        void expander_Collapsed(object sender, RoutedEventArgs e)
        {
            Grid imageGrid = ((Expander)sender).Parent as Grid;

            imageGrid.RowDefinitions.ElementAt(0).Height = new GridLength(.70, GridUnitType.Star);
            imageGrid.RowDefinitions.ElementAt(1).Height = new GridLength(.15, GridUnitType.Star);
            imageGrid.RowDefinitions.ElementAt(2).Height = new GridLength(.15, GridUnitType.Star);
        }

        void expander_Expanded(object sender, RoutedEventArgs e)
        {
            Grid imageGrid = ((Expander)sender).Parent as Grid;

            imageGrid.RowDefinitions.ElementAt(0).Height = new GridLength(.55, GridUnitType.Star);
            imageGrid.RowDefinitions.ElementAt(1).Height = new GridLength(.15, GridUnitType.Star);
            imageGrid.RowDefinitions.ElementAt(2).Height = new GridLength(.30, GridUnitType.Star);
        }

        async void checkBox_Unchecked(object sender, RoutedEventArgs e)
        {
            int idx = Convert.ToInt32(((Grid)((CheckBox)sender).Parent).Tag);

            for (int i = 0; i < bottomStack.Children.Count; ++i)
            {
                int bottomIdx = Convert.ToInt32(((Image)bottomStack.Children[i]).Tag);
                if (bottomIdx == idx)
                {
                    //MyStyles.FadeOut((Image)bottomStack.Children[i], new Duration(new TimeSpan(0, 0, 1)));
                    //ScaleDown((Image)bottomStack.Children[i]);

                    //await Task.Delay(1025);

                    while (((Image)bottomStack.Children[i]).Height > 5)
                    {
                        ((Image)bottomStack.Children[i]).Height -= 5;
                        await Task.Delay(3);
                    }

                    for (int j = 0; j < bottomStack.Children.Count; ++j)
                    {
                        int bottomIdx2 = Convert.ToInt32(((Image)bottomStack.Children[j]).Tag);
                        if (bottomIdx2 == idx)
                            bottomStack.Children.RemoveAt(j);
                    }
                }
            }
        }

        bool acscale = true;

        void checkBox_Checked(object sender, RoutedEventArgs e)
        {
            int idx = Convert.ToInt32(((Grid)((CheckBox)sender).Parent).Tag);


            for (int i = 0; i < bottomStack.Children.Count; ++i)
            {
                int bottomIdx = Convert.ToInt32(((Image)bottomStack.Children[i]).Tag);
                if (bottomIdx == idx)
                {
                    ((Image)bottomStack.Children[i]).BeginAnimation(UIElement.OpacityProperty, null);
                    return;
                }
            }

            //Grid imageGrid = new Grid();
            //for (int j = 0; j < 2; ++j)
            //{
            //    RowDefinition row = new RowDefinition();
            //    imageGrid.RowDefinitions.Add(row);
            //}
            //imageGrid.ShowGridLines = true;

            //imageGrid.RowDefinitions.ElementAt(0).Height = new GridLength(.85, GridUnitType.Star);
            //imageGrid.RowDefinitions.ElementAt(1).Height = new GridLength(.15, GridUnitType.Star);

            Image imgStack = new Image();
            imgStack.Source = MyData.bitMaps[idx];
            imgStack.Stretch = Stretch.Fill;
            imgStack.Tag = "" + idx;
            imgStack.Height = 0;

            if (acscale)
            {
                //imgStack.Height = topScroll.ExtentHeight * 0.5;

                ////var fadeInAnimation = new DoubleAnimation(1, 0.0, new Duration(new TimeSpan(0, 0, 1)));
                ////imgStack.BeginAnimation(UIElement.OpacityProperty, fadeInAnimation);

                //Storyboard storyboard = new Storyboard();

                //ScaleTransform scale = new ScaleTransform(1, 1);
                //imgStack.RenderTransformOrigin = new Point(0.5, 0.5);
                //imgStack.RenderTransform = scale;

                //DoubleAnimation growAnimation = new DoubleAnimation();
                //growAnimation.Duration = TimeSpan.FromMilliseconds(750);
                //growAnimation.From = 0.1;
                //growAnimation.To = 1;
                //storyboard.Children.Add(growAnimation);

                //Storyboard.SetTargetProperty(growAnimation, new PropertyPath("RenderTransform.ScaleX"));
                ////Storyboard.SetTargetProperty(growAnimation, new PropertyPath("RenderTransform.ScaleY"));
                //Storyboard.SetTarget(growAnimation, imgStack);

                //storyboard.Begin();

                ScaleUp(imgStack);

                //acscale = false;
            }
            else
            {

                acscale = true;
            }

            //MyStyles.FadeIn(imgStack, new Duration(new TimeSpan(0, 0, 1)));
            
            //Grid.SetRow(imgStack, 0);
            //Grid.SetRowSpan(imgStack, 2);
            //imageGrid.Children.Add(imgStack);

            //Grid.SetRow(imgStack, 1);
            //imageGrid.Children.Add(imgStack);

            bottomStack.Children.Add(imgStack);

            if (bottomStack.Children.Count > 1)
            {
                for (int i = bottomStack.Children.Count - 2; i >= 0; --i)
                {
                    UIElement elem = bottomStack.Children[i];
                    bottomStack.Children.Remove(elem);
                    bottomStack.Children.Insert(i + 1, elem);
                }
                bottomStack.Children.Remove(imgStack);
                bottomStack.Children.Insert(0, imgStack);
            }
        }


        async void ScaleUp(Image elem){
            FrameworkElement fe = elem as FrameworkElement;
            if (fe == null) return;
            //((FrameworkElement)elem).Height = topScroll.ExtentHeight * 0.1;

            //elem.aHeight = topScroll.ExtentHeight * 0.1;

            while (((FrameworkElement)elem).Height < 300)
            {
                ((FrameworkElement)elem).Height += 5;
                await Task.Delay(5);
            }

            //int w = 5;
            //while (w < MyData.bitMaps[0].Width)
            //{
            //    Image croppedImage = new Image();
            //    croppedImage.Width = w;
            //    //croppedImage.Margin = new Thickness(5);

            //    // Create a CroppedBitmap based off of a xaml defined resource.
            //    CroppedBitmap cb = new CroppedBitmap(
            //       (BitmapSource)MyData.bitMaps[0],
            //       new Int32Rect(
            //           ((int)MyData.bitMaps[0].Width)-w, 
            //           0, 
            //           w,
            //           (int)topScroll.ExtentHeight));//((int)MyData.bitMaps[0].Height)));       //select region rect
            //    //elem.Width = w;


            //    croppedImage.Source = cb;                 //set image source to cropped
            //    croppedImage.Stretch = Stretch.Fill;

            //    //resi

            //    if(bottomStack.Children.Count > 0)
            //        bottomStack.Children.RemoveAt(0);

            //    bottomStack.Children.Insert(0, croppedImage);

            //    w += 5;
            //    await Task.Delay(5);
            //}
        }

        async void ScaleDown(Image elem)
        {
            FrameworkElement fe = elem as FrameworkElement;
            if (fe == null) return;
            //((FrameworkElement)elem).Height = topScroll.ExtentHeight * 0.1;

            //elem.aHeight = topScroll.ExtentHeight * 0.1;

            while (((FrameworkElement)elem).Height > topScroll.ExtentHeight * 0.1)
            {
                ((FrameworkElement)elem).Height -= 5;
                await Task.Delay(500);
            }
        }
    }
}
