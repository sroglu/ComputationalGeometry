using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace CompGeo.Samples
{
    /// <summary>
    /// Binds the runtime uGUI panel to the <see cref="Workbench"/> and its modes — the new-core form of
    /// the original <c>UI.cs</c>/<c>UI2.cs</c> <c>BindEvents</c>. Controls are located by GameObject name
    /// (built once by the editor UI builder), then wired with runtime listeners. The shared top bar
    /// (mesh dropdown + load, vertex/edge/surface toggles, reset-camera) is always wired; the geodesics
    /// algorithm dropdown and the unfold toggle are wired only when those modes/controls are present.
    /// </summary>
    public sealed class WorkbenchUI : MonoBehaviour
    {
        Workbench _wb;
        WorkbenchCamera _cam;
        GeodesicsMode _geodesics;
        UnfoldMode _unfold;

        void Start()
        {
            _wb = FindFirstObjectByType<Workbench>();
            _cam = FindFirstObjectByType<WorkbenchCamera>();
            _geodesics = FindFirstObjectByType<GeodesicsMode>();
            _unfold = FindFirstObjectByType<UnfoldMode>();

            // Shared top bar.
            Dropdown meshDropdown = Find<Dropdown>("MeshDropdown");
            if (meshDropdown != null)
            {
                meshDropdown.ClearOptions();
                meshDropdown.AddOptions(new List<string>(_wb.Catalog.Names()));
                meshDropdown.SetValueWithoutNotify(_wb.SelectedIndex);
                meshDropdown.onValueChanged.AddListener(i => _wb.LoadMesh(i));
            }

            Button load = Find<Button>("LoadButton");
            if (load != null) load.onClick.AddListener(() => _wb.LoadMesh(meshDropdown.value));

            BindToggle("VertexToggle", _wb.showVertices, _wb.SetShowVertices);
            BindToggle("EdgeToggle", _wb.showEdges, _wb.SetShowEdges);
            BindToggle("SurfaceToggle", _wb.showSurface, _wb.SetShowSurface);

            Button reset = Find<Button>("ResetCamButton");
            if (reset != null && _cam != null) reset.onClick.AddListener(_cam.ResetView);

            // Geodesics-only.
            if (_geodesics != null)
            {
                Dropdown algo = Find<Dropdown>("AlgoDropdown");
                if (algo != null)
                {
                    algo.ClearOptions();
                    algo.AddOptions(new List<string> { "Dijkstra", "A*" });
                    algo.SetValueWithoutNotify((int)_geodesics.algorithm);
                    algo.onValueChanged.AddListener(_geodesics.SetAlgorithm);
                }

                Slider widthSlider = Find<Slider>("PathWidthSlider");
                if (widthSlider != null)
                {
                    widthSlider.minValue = 0.0008f;
                    widthSlider.maxValue = 0.012f;
                    widthSlider.SetValueWithoutNotify(_geodesics.pathWidth);
                    widthSlider.onValueChanged.AddListener(_geodesics.SetPathWidth);
                }

                Text distanceLabel = Find<Text>("DistanceLabel");
                if (distanceLabel != null)
                {
                    void RefreshDistance()
                    {
                        distanceLabel.text = _geodesics.Target < 0
                            ? $"source: {_geodesics.Source}\nright-click a target"
                            : $"{_geodesics.Source} → {_geodesics.Target}\ndistance: {_geodesics.LastCost:F3}";
                    }
                    _geodesics.PathUpdated += RefreshDistance;
                    RefreshDistance();
                }
            }

            // Unfold-only.
            if (_unfold != null)
            {
                Toggle morph = Find<Toggle>("MorphToggle");
                if (morph != null)
                {
                    morph.SetIsOnWithoutNotify(_unfold.morphing);
                    morph.onValueChanged.AddListener(_unfold.SetMorphing);
                }

                Button fold = Find<Button>("FoldButton");
                if (fold != null) fold.onClick.AddListener(_unfold.ToggleFold);
            }
        }

        void BindToggle(string name, bool initial, UnityEngine.Events.UnityAction<bool> setter)
        {
            Toggle t = Find<Toggle>(name);
            if (t == null) return;
            t.SetIsOnWithoutNotify(initial);
            t.onValueChanged.AddListener(setter);
        }

        T Find<T>(string objectName) where T : Component
        {
            foreach (var c in GetComponentsInChildren<T>(true))
                if (c.gameObject.name == objectName) return c;
            return null;
        }
    }
}
