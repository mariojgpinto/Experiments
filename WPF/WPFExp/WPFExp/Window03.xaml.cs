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
    /// Interaction logic for Window03.xaml
    /// </summary>
    public partial class Window03 : Window
    {
        StackPanel myStackPanel;
        ScrollViewer myScrollViewerStack;

        DockPanel myDockPanel;
        ScrollViewer myScrollViewerDock;

        public Window03()
        {
            InitializeComponent();

            RowDefinition myRowDef1 = new RowDefinition();
            RowDefinition myRowDef2 = new RowDefinition();
            SliderGrid.RowDefinitions.Add(myRowDef1);
            SliderGrid.RowDefinitions.Add(myRowDef2);




            // Define the DockPanel
            myScrollViewerDock = new ScrollViewer();
            myScrollViewerDock.VerticalScrollBarVisibility = ScrollBarVisibility.Visible;
            myScrollViewerDock.HorizontalScrollBarVisibility = ScrollBarVisibility.Hidden;
            myScrollViewerDock.PreviewMouseWheel += myScrollViewerDock_MouseWheel;
            //myScrollViewerDock.MouseWheel

            myDockPanel = new DockPanel();

            myScrollViewerDock.Content = myDockPanel;

            Grid.SetRow(myScrollViewerDock, 0);
            SliderGrid.Children.Add(myScrollViewerDock);



            

            // Define the StackPanel
            myScrollViewerStack = new ScrollViewer();
            myScrollViewerStack.VerticalScrollBarVisibility = ScrollBarVisibility.Hidden;
            myScrollViewerStack.HorizontalScrollBarVisibility = ScrollBarVisibility.Disabled;
            

            myStackPanel = new StackPanel();
            myStackPanel.Orientation = Orientation.Vertical;

            myScrollViewerStack.Content = myStackPanel;

            Grid.SetRow(myScrollViewerStack, 1);
            SliderGrid.Children.Add(myScrollViewerStack);




            int counter = 0;
            for (int i = 0; i < MyData.files.Count() ; ++i)
            {
                Image imgDock = new Image();
                imgDock.Source = MyData.bitMaps[counter];
                imgDock.Stretch = Stretch.Uniform;

                myDockPanel.Children.Add(imgDock);




                Image imgStack= new Image();
                imgStack.Source = MyData.bitMaps[counter];
                imgStack.Stretch = Stretch.Uniform;

                myStackPanel.Children.Add(imgStack);




                counter++;
            }
        }

        void myScrollViewerDock_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            ScrollViewer scrollviewer = sender as ScrollViewer;
            if (e.Delta > 0)
            {
                scrollviewer.LineLeft();
            }
            else
                scrollviewer.LineRight();
            e.Handled = true;
        }
    }
}
