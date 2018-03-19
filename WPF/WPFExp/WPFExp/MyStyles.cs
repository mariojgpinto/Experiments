using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;

namespace WPFExp
{
    class MyStyles
    {
        public static void FadeIn(UIElement elem, Duration duration){
            var fadeInAnimation = new DoubleAnimation(0.0, 1.0, duration);
            elem.BeginAnimation(UIElement.OpacityProperty, fadeInAnimation);
        }

        public static void FadeOut(UIElement elem, Duration duration)
        {
            var fadeInAnimation = new DoubleAnimation(elem.Opacity, 0.0, duration);
            elem.BeginAnimation(UIElement.OpacityProperty, fadeInAnimation);
        }
    }
}
