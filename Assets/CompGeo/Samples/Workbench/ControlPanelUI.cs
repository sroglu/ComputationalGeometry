using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.UI;
using CompGeo.Core;
using CompGeo.MeshProcessing;

namespace CompGeo.Samples
{
    /// <summary>
    /// The unified Control Panel binder — the new-core form of the original homework's <c>UI.cs</c>
    /// <c>BindEvents</c>. Locates the baked uGUI controls by name and wires them to the <see cref="Workbench"/>
    /// and <see cref="GeodesicsMode"/>: Model Features (vertices / edges / normals), Mesh Operations
    /// (dropdown + Load + Remesh), and Search Shortest Path (algorithm, Source/Dest with Pick buttons,
    /// Find Random, Search), plus Reset Camera. Mirrors the original panel one-to-one; picking goes through
    /// the KD-tree (no GameObject-per-vertex raycast).
    /// </summary>
    public sealed class ControlPanelUI : MonoBehaviour
    {
        Workbench _wb;
        WorkbenchCamera _cam;
        GeodesicsMode _geo;
        InputField _source, _dest;
        Toggle _random;
        Text _status;

        void Start()
        {
            _wb = FindFirstObjectByType<Workbench>();
            _cam = FindFirstObjectByType<WorkbenchCamera>();
            _geo = FindFirstObjectByType<GeodesicsMode>();

            // Model Features (each bound only if that toggle is present in the scene's panel).
            BindToggle("VertexToggle", _wb.showVertices, _wb.SetShowVertices);
            BindToggle("EdgeToggle", _wb.showEdges, _wb.SetShowEdges);
            BindToggle("SurfaceToggle", _wb.showSurface, _wb.SetShowSurface);
            BindToggle("NormalsToggle", _wb.showNormals, _wb.SetShowNormals);

            // Mesh Operations.
            Dropdown mesh = Find<Dropdown>("MeshDropdown");
            if (mesh != null)
            {
                mesh.ClearOptions();
                mesh.AddOptions(new List<string>(_wb.Catalog.Names()));
                mesh.SetValueWithoutNotify(_wb.SelectedIndex);
                mesh.onValueChanged.AddListener(_wb.LoadMesh);
            }
            Bind("LoadButton", () => { if (mesh != null) _wb.LoadMesh(mesh.value); });
            Bind("RemeshButton", _wb.Remesh);

            Dropdown rmode = Find<Dropdown>("MethodDropdown");
            if (rmode != null)
            {
                rmode.ClearOptions();
                rmode.AddOptions(new List<string> { "Original (homework)", "Improved (mutual-agreement)" });
                rmode.SetValueWithoutNotify((int)_wb.remeshMode);
                rmode.onValueChanged.AddListener(i => _wb.remeshMode = (RemeshMode)i);
            }

            Dropdown umethod = Find<Dropdown>("UnfoldMethodDropdown");
            if (umethod != null)
            {
                umethod.ClearOptions();
                umethod.AddOptions(new List<string> { "Original (homework)", "Tutte (performant)" });
                umethod.SetValueWithoutNotify((int)_wb.unfoldMethod);
                umethod.onValueChanged.AddListener(i => _wb.unfoldMethod = (UnfoldMethod)i);
            }
            Bind("UnfoldButton", _wb.Unfold);

            // Search Shortest Path.
            if (_geo != null)
            {
                Dropdown algo = Find<Dropdown>("AlgoDropdown");
                if (algo != null)
                {
                    algo.ClearOptions();
                    algo.AddOptions(new List<string> { "Dijkstra", "A*" });
                    algo.SetValueWithoutNotify((int)_geo.algorithm);
                    algo.onValueChanged.AddListener(_geo.SetAlgorithm);
                }

                _source = Find<InputField>("SourceInput");
                _dest = Find<InputField>("DestInput");
                _random = Find<Toggle>("RandomToggle");
                _status = Find<Text>("DistanceLabel");

                if (_source != null) _source.onEndEdit.AddListener(s => { if (TryParse(s, out int v)) _geo.SetSource(v); });
                if (_dest != null) _dest.onEndEdit.AddListener(s => { if (TryParse(s, out int v)) _geo.SetTarget(v); });

                Bind("PickSourceButton", _geo.ArmPickSource);
                Bind("PickDestButton", _geo.ArmPickDest);
                Bind("SearchButton", () =>
                {
                    if (_random != null && _random.isOn) _geo.SearchRandom();
                    else _geo.Search();
                });

                Slider width = Find<Slider>("PathWidthSlider");
                if (width != null)
                {
                    width.minValue = 0.0008f;
                    width.maxValue = 0.012f;
                    width.SetValueWithoutNotify(_geo.pathWidth);
                    width.onValueChanged.AddListener(_geo.SetPathWidth);
                }

                _geo.PathUpdated += RefreshReadout;
                RefreshReadout();
            }

            if (_cam != null) Bind("ResetCamButton", _cam.ResetView);
        }

        void RefreshReadout()
        {
            if (_source != null) _source.SetTextWithoutNotify(_geo.Source.ToString(CultureInfo.InvariantCulture));
            if (_dest != null) _dest.SetTextWithoutNotify(_geo.Target.ToString(CultureInfo.InvariantCulture));
            if (_status != null)
                _status.text = _geo.Target < 0
                    ? $"source {_geo.Source}\npick a destination"
                    : $"{_geo.Source} → {_geo.Target}\ndistance: {_geo.LastCost:F3}";
        }

        void Bind(string name, UnityEngine.Events.UnityAction action)
        {
            Button b = Find<Button>(name);
            if (b != null) b.onClick.AddListener(action);
        }

        void BindToggle(string name, bool initial, UnityEngine.Events.UnityAction<bool> setter)
        {
            Toggle t = Find<Toggle>(name);
            if (t == null) return;
            t.SetIsOnWithoutNotify(initial);
            t.onValueChanged.AddListener(setter);
        }

        static bool TryParse(string s, out int v) => int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out v);

        T Find<T>(string objectName) where T : Component
        {
            foreach (var c in GetComponentsInChildren<T>(true))
                if (c.gameObject.name == objectName) return c;
            return null;
        }
    }
}
