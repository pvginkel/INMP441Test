using System.Windows;
using System.Windows.Media;
using Color = System.Windows.Media.Color;

namespace Receiver;

/// <summary>
/// Base class for WPF windows.
/// </summary>
/// <remarks>
/// Use this class as the base class for any WPF windows you create to
/// integrate with the TQL theme and styling.
/// </remarks>
public class BaseWindow : Window
{
    /// <summary>
    /// Initializes a new <see cref="BaseWindow"/>.
    /// </summary>
    public BaseWindow()
    {
        TextOptions.SetTextFormattingMode(this, TextFormattingMode.Display);
        UseLayoutRounding = true;
        SnapsToDevicePixels = true;
        ShowInTaskbar = false;
        RenderOptions.SetClearTypeHint(this, ClearTypeHint.Enabled);
        Style = (Style)FindResource("CustomWindowStyle");
    }
}
