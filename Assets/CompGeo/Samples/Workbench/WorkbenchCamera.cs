using UnityEngine;
using UnityEngine.EventSystems;

namespace CompGeo.Samples
{
    /// <summary>
    /// Mouse camera control for the sample workbench (replaces the original UI.cs "MouseControlZone"):
    /// <b>left-drag</b> orbits around the pivot, <b>right-drag</b> pans, the <b>scroll wheel</b> dollies,
    /// and <see cref="ResetView"/> restores the start pose. Input is ignored while the pointer is over a
    /// UI element so dragging on the panel never moves the camera. Uses the legacy Input Manager for now.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public sealed class WorkbenchCamera : MonoBehaviour
    {
        public Vector3 pivot = Vector3.zero;
        public float orbitSpeed = 5f;
        public float panSpeed = 0.4f;
        public float zoomSpeed = 0.5f;

        Vector3 _initialPos;
        Quaternion _initialRot;
        Vector3 _pivot;

        void Start()
        {
            _initialPos = transform.position;
            _initialRot = transform.rotation;
            _pivot = pivot;
        }

        void Update()
        {
            if (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject())
                return;

            float mx = Input.GetAxis("Mouse X");
            float my = Input.GetAxis("Mouse Y");

            if (Input.GetMouseButton(0)) // orbit
            {
                transform.RotateAround(_pivot, Vector3.up, mx * orbitSpeed);
                transform.RotateAround(_pivot, transform.right, -my * orbitSpeed);
            }
            else if (Input.GetMouseButton(1)) // pan (move camera and pivot together)
            {
                Vector3 delta = (-mx * transform.right - my * transform.up) * panSpeed;
                transform.position += delta;
                _pivot += delta;
            }

            float scroll = Input.mouseScrollDelta.y;
            if (scroll != 0f)
                transform.position += transform.forward * (scroll * zoomSpeed);
        }

        public void ResetView()
        {
            transform.position = _initialPos;
            transform.rotation = _initialRot;
            _pivot = pivot;
        }
    }
}
