using UnityEngine;
using UnityEngine.UI;

namespace CompGeo.Samples
{
    /// <summary>
    /// Binds the Shape Completion control panel (uGUI, built into the scene) to <see cref="ShapeCompletionDemo"/>
    /// by control name — the same pattern as <see cref="ControlPanelUI"/>, replacing the old IMGUI box.
    /// </summary>
    public sealed class ShapeCompletionUI : MonoBehaviour
    {
        void Start()
        {
            var demo = FindFirstObjectByType<ShapeCompletionDemo>();
            if (demo == null) return;

            Bind("CompleteButton", demo.Complete);
            Bind("ResetButton", demo.ResetShape);

            Slider slider = Find<Slider>("DotSlider");
            if (slider != null)
            {
                slider.minValue = 1;
                slider.maxValue = 64;
                slider.wholeNumbers = true;
                slider.SetValueWithoutNotify(demo.ExtraPoints);
                slider.onValueChanged.AddListener(demo.SetExtraPoints);
            }

            Text status = Find<Text>("StatusLabel");
            if (status != null)
            {
                status.text = "";
                demo.StatusChanged += s => status.text = s;
            }
        }

        void Bind(string name, UnityEngine.Events.UnityAction action)
        {
            Button b = Find<Button>(name);
            if (b != null) b.onClick.AddListener(action);
        }

        T Find<T>(string objectName) where T : Component
        {
            foreach (var c in GetComponentsInChildren<T>(true))
                if (c.gameObject.name == objectName) return c;
            return null;
        }
    }
}
