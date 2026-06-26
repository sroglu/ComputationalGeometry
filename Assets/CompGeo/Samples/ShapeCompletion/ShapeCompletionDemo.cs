using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using CompGeo.Core;

namespace CompGeo.Samples
{
    /// <summary>
    /// 2D shape-completion demo (the new-core form of the original CENG789 <c>ShapeCompletion</c>): an open
    /// chain of points is drawn with a <see cref="LineRenderer"/>; pressing <b>Complete</b> closes it by
    /// appending an arc from <see cref="ShapeCompletion.CompleteArc"/> and joining back to the start.
    /// The chain is inspector-editable; <b>Reset</b> restores the open outline. All drawing is immediate —
    /// no scene wiring beyond this component.
    /// </summary>
    [RequireComponent(typeof(LineRenderer))]
    public sealed class ShapeCompletionDemo : MonoBehaviour
    {
        [Tooltip("Open boundary chain (XY plane). Completion fills the gap from the last point back to the first.")]
        public List<Vector2> openPoints = new List<Vector2>
        {
            new Vector2(1f, -3f), new Vector2(-0.5f, -4f), new Vector2(-2f, -3f), new Vector2(-3f, -1f),
            new Vector2(-1.5f, 1.5f), new Vector2(0.5f, 2f), new Vector2(2.5f, 1f), new Vector2(3f, -0.5f),
        };

        [Tooltip("How many points to synthesise along the closing arc.")]
        [Range(1, 64)] public int extraPoints = 10;

        public Color lineColor = new Color(0.1f, 0.9f, 1f, 1f);
        public float lineWidth = 0.06f;

        /// <summary>Raised after Complete/Reset with a short status line (the UI shows it).</summary>
        public event System.Action<string> StatusChanged;
        public int ExtraPoints => extraPoints;

        LineRenderer _line;
        readonly List<Vector2> _current = new List<Vector2>();
        bool _completed;

        void Awake() => _line = GetComponent<LineRenderer>();

        void Start()
        {
            _line.useWorldSpace = false;
            _line.widthMultiplier = lineWidth;
            _line.numCornerVertices = 2;
            if (_line.sharedMaterial == null)
                _line.material = new Material(Shader.Find("Sprites/Default"));
            _line.startColor = _line.endColor = lineColor;
            ResetShape();
        }

        /// <summary>Restore the open chain (drops any completion).</summary>
        public void ResetShape()
        {
            _completed = false;
            _current.Clear();
            _current.AddRange(openPoints);
            Redraw(false);
            StatusChanged?.Invoke($"{_current.Count} open points");
        }

        /// <summary>Close the current open chain with a synthesised arc, then join back to the start.</summary>
        public void Complete()
        {
            if (_completed || _current.Count < 2) return;

            var open = new float2[_current.Count];
            for (int i = 0; i < _current.Count; i++) open[i] = _current[i];

            float2[] arc = ShapeCompletion.CompleteArc(open, extraPoints);
            foreach (float2 p in arc) _current.Add(p);

            _completed = true;
            Redraw(true);
            StatusChanged?.Invoke($"{_current.Count} points, closed (+{extraPoints})");
        }

        /// <summary>Bound to the UI slider: set how many points the closing arc synthesises.</summary>
        public void SetExtraPoints(float v) => extraPoints = Mathf.Clamp(Mathf.RoundToInt(v), 1, 64);

        void Redraw(bool closed)
        {
            _line.loop = closed;
            _line.positionCount = _current.Count;
            for (int i = 0; i < _current.Count; i++)
                _line.SetPosition(i, new Vector3(_current[i].x, _current[i].y, 0f));
        }
    }
}
