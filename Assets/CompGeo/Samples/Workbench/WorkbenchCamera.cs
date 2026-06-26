using UnityEngine;
using UnityEngine.EventSystems;

namespace CompGeo.Samples
{
    /// <summary>
    /// Model-inspector camera: the <b>object spins in place around fixed WORLD axes</b> (left-drag →
    /// horizontal = world Y, vertical = world X), so the rotation is the same no matter how the model is
    /// already turned — never local. The camera itself keeps a fixed viewing direction; the <b>scroll
    /// wheel</b> dollies and <b>right/middle-drag</b> pans. Input is ignored over UI.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public sealed class WorkbenchCamera : MonoBehaviour
    {
        [Tooltip("Transform spun by the mouse (defaults to the Workbench). Rotated around WORLD axes.")]
        public Transform objectTransform;
        public Vector3 target = Vector3.zero;

        [Header("Sensitivity (tune live in Play)")]
        public float orbitSensitivity = 0.3f;   // degrees per pixel
        public float zoomSensitivity = 3f;       // distance per scroll notch
        public float panSensitivity = 0.0015f;   // world per pixel, per unit distance

        [Header("Limits")]
        public float minDistance = 0.6f;
        public float maxDistance = 20f;

        Vector3 _viewDir;
        float _distance;
        Vector3 _target;
        Vector3 _lastMouse;
        Quaternion _objRot0;
        Vector3 _target0;
        float _distance0;

        void Start()
        {
            if (objectTransform == null)
            {
                var wb = FindFirstObjectByType<Workbench>();
                if (wb != null) objectTransform = wb.transform;
            }

            _target = target;
            Vector3 toTarget = _target - transform.position;
            _distance = Mathf.Clamp(toTarget.magnitude, minDistance, maxDistance);
            _viewDir = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : Vector3.forward;
            _lastMouse = Input.mousePosition;

            _objRot0 = objectTransform != null ? objectTransform.rotation : Quaternion.identity;
            _target0 = _target;
            _distance0 = _distance;
            ApplyCamera();
        }

        void LateUpdate()
        {
            Vector3 mouse = Input.mousePosition;
            Vector3 delta = mouse - _lastMouse;
            _lastMouse = mouse;

            if (EventSystem.current == null || !EventSystem.current.IsPointerOverGameObject())
            {
                if (Input.GetMouseButton(0) && objectTransform != null)
                {
                    // Arcball / trackball: rotate the object around the CAMERA's screen axes — horizontal
                    // drag spins it around the screen-vertical (camera up), vertical drag around the
                    // screen-horizontal (camera right). Pre-multiplying applies it in world space, so it
                    // feels like grabbing the object and turning it, intuitively, from any angle.
                    Quaternion rot = Quaternion.AngleAxis(-delta.x * orbitSensitivity, transform.up)
                                   * Quaternion.AngleAxis(delta.y * orbitSensitivity, transform.right);
                    objectTransform.rotation = rot * objectTransform.rotation;
                }
                else if (Input.GetMouseButton(1) || Input.GetMouseButton(2))
                {
                    _target -= (transform.right * delta.x + transform.up * delta.y) * (panSensitivity * _distance);
                }

                float scroll = Input.GetAxis("Mouse ScrollWheel");
                if (scroll != 0f)
                    _distance = Mathf.Clamp(_distance - scroll * zoomSensitivity, minDistance, maxDistance);
            }

            ApplyCamera();
        }

        void ApplyCamera()
        {
            transform.rotation = Quaternion.LookRotation(_viewDir, Vector3.up);
            transform.position = _target - _viewDir * _distance;
        }

        public void ResetView()
        {
            if (objectTransform != null) objectTransform.rotation = _objRot0;
            _target = _target0;
            _distance = _distance0;
            ApplyCamera();
        }
    }
}
