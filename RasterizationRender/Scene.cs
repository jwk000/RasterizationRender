using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RasterizationRender
{
    public class Scene
    {
        //场景里的物体
        public List<GameObject> mObjects = new List<GameObject>();
        //摄像机
        public GameObject mCamera;

        public List<Plane> mPlanes = new List<Plane>();


        public Bitmap mImage;

        public Scene()
        {

        }

        public void AddCamera(GameObject cam)
        {
            mCamera = cam;
        }
        public void AddObject(GameObject obj)
        {
            mObjects.Add(obj);
        }

        //渲染物体
        public void Render()
        {
            List<GameObject> clipObjects = new List<GameObject>();
            foreach(GameObject go in mObjects)
            {
               GameObject co = ClipObject(go);
                if (co != null)
                {
                    clipObjects.Add(co);
                }
            }

            foreach(GameObject go in clipObjects)
            {
                RenderObject(go);
            }
        }

        GameObject ClipObject(GameObject go)
        {
            foreach(Plane p in mPlanes)
            {
                go = ClipByPlane(p, go);
                if (go == null)
                {
                    return null;
                }
            }
            return go;
        }

        GameObject ClipByPlane(Plane p,GameObject go)
        {
            var mesh = go.GetComponent<Mesh>();
            var bs = mesh.GetBoundingSphare();
            float d = SignedDistance(p, bs.center);
            if (d > bs.r)//视野内
            {
                return go;
            }
            if (d < -bs.r)//视野外
            {
                return null;
            }
            //todo
            return go;
        }

        float SignedDistance(Plane p, Vector3 v)
        {
            return v.X * p.Normal.X + v.Y * p.Normal.Y + v.Z + p.Normal.Z + p.D;
        }

        void RenderObject(GameObject go)
        {
            Canvas c = mCamera.GetComponent<Canvas>();
            Transform ct = mCamera.GetComponent<Transform>();
            Mesh m = go.GetComponent<Mesh>();
            if (m != null)
            {
                Transform t = go.GetComponent<Transform>();
                Matrix4x4 model = t.MakeYRotationMatrix() * t.MakeScaleMatrix() * t.MakeTranslationMatrix();
                Matrix4x4 view = ct.MakeTranslationMatrix()*ct.MakeYRotationMatrix();
                List<Vector3> trans = new List<Vector3>();
                foreach(var v in m.vertices)
                {
                    Vector4 v4 = new Vector4(v, 1);
                    Vector4 vt = Multiply(model, v4);
                    trans.Add(new Vector3(vt.X,vt.Y,vt.Z));
                }
                c.DrawWireframeTriangles(trans, m.triangles);
            }
        }

        Vector4 Multiply(Matrix4x4 m4, Vector4 v4)
        {
            return new Vector4(
                m4.M11 * v4.X + m4.M12 * v4.Y + m4.M13 * v4.Z + m4.M14 * v4.W,
                m4.M21 * v4.X + m4.M22 * v4.Y + m4.M23 * v4.Z + m4.M24 * v4.W,
                m4.M31 * v4.X + m4.M32 * v4.Y + m4.M33 * v4.Z + m4.M34 * v4.W,
                m4.M41 * v4.X + m4.M42 * v4.Y + m4.M43 * v4.Z + m4.M44 * v4.W
                );
        }

        public void Awake()
        {
            foreach (var obj in mObjects)
            {
                obj.Awake();
            }
        }


        public void Start()
        {
            foreach (var obj in mObjects)
            {
                obj.Start();
            }
        }


        public void Update()
        {
            foreach (var obj in mObjects)
            {
                obj.Update();
            }

            Render();
        }

        public void Destroy()
        {
            foreach (var obj in mObjects)
            {
                obj.Destroy();
            }
        }
    }
}
