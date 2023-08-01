using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RasterizationRender
{


    public class Vertex
    {
        public Vector3 Pos;
        public Color Color;
    }
    public struct Triangle
    {
        public int v0;
        public int v1;
        public int v2;


        public Triangle(int v0, int v1, int v2)
        {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
        }
    }

    //包围球
    public class BoundingSphare
    {
        public float Radius;
        public Vector3 Pos;
    }
    public class Mesh
    {
        public List<Vector3> Verteics; //顶点
        public List<Triangle> Trangles;//三角形
        public List<Vector3> Normals;//法线

        //所有顶点距离原点最近点和最远点为直径
        public BoundingSphare GetBoundingSphare()
        {
            BoundingSphare bs = new BoundingSphare();
            Vector3 near = Verteics[0], far = Verteics[0];
            foreach (var v in Verteics)
            {
                if (v.Length() < near.Length())
                {
                    near = v;
                }
                if (v.Length() > far.Length())
                {
                    far = v;
                }
            }

            Vector3 d = far - near;
            bs.Pos = near + d / 2;
            bs.Radius = d.Length() / 2;
            return bs;
        }
    }

    public class Transform
    {
        public Vector3 Position;
        public Vector3 Rotation;
        public Vector3 Scale;

        public Matrix4x4 MakeTranslationMatrix()
        {
            return new Matrix4x4(
                1, 0, 0, Position.X,
                0, 1, 0, Position.Y,
                0, 0, 1, Position.Z,
                0, 0, 0, 1);
        }

        public Matrix4x4 MakeXRotationMatrix()
        {
            float cos = MathF.Cos(Rotation.X * MathF.PI / 180);
            float sin = MathF.Sin(Rotation.X * MathF.PI / 180);
            return new Matrix4x4(
                1, 0, 0, 0,
                0, cos, sin, 0,
                0, sin, cos, 0,
                0, 0, 0, 1);
        }

        public Matrix4x4 MakeYRotationMatrix()
        {
            float cos = MathF.Cos(Rotation.Y * MathF.PI / 180);
            float sin = MathF.Sin(Rotation.Y * MathF.PI / 180);
            return new Matrix4x4(
                cos, 0, sin, 0,
                0, 1, 0, 0,
                -sin, 0, cos, 0,
                0, 0, 0, 1);
        }

        public Matrix4x4 MakeZRotationMatrix()
        {
            float cos = MathF.Cos(Rotation.Z * MathF.PI / 180);
            float sin = MathF.Sin(Rotation.Z * MathF.PI / 180);
            return new Matrix4x4(
                 cos, -sin, 0, 0,
               sin, cos, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
        }

        public Matrix4x4 MakeScaleMatrix()
        {
            return new Matrix4x4(
                Scale.X, 0, 0, 1,
                0, Scale.Y, 0, 1,
                0, 0, Scale.Z, 1,
                0, 0, 0, 1);
        }
    }

    public class Material
    {
        public Color Color = Color.Pink;//固有色
        public int Specular = 500;//高光系数
        public float Reflective = 0;//反射系数 0没有反射 1完全反射
        public float Opacity = 1;//不透明度 0完全透明 1完全不透明
        public float Refraction = 1;//折射率 空气的折射率是1 水和玻璃是1.33、
    }

    public class GameObject
    {
        public Mesh Mesh;
        public Material Material;
        public Transform Transform;
    }

    public class Canvas
    {

        public float Width;
        public float Height;
        public Bitmap Bitmap;
        public Canvas(int w, int h)
        {
            Width = w;
            Height = h;
            Bitmap = new Bitmap(w, h);
        }

        //近大远小
        public Vector2 WorldToViewPort(Vector3 world)
        {
            return new Vector2(world.X / world.Z, world.Y / world.Z);
        }
        //(-1,1) --> (0,w)
        //(-1,1) --> (0,h)
        public Vector2 ViewPortToScreen(Vector2 viewport)
        {
            return new Vector2((viewport.X + 1) / 2 * Width, (viewport.Y + 1) / 2 * Height);
        }

        public Vector2 WorldToScreen(Vector3 world)
        {
            return ViewPortToScreen(WorldToViewPort(world));
        }

        //(0,w) --> (-1,1)
        public Vector2 ScreenToViewPort(Vector2 screen)
        {
            return new Vector2(screen.X / Width * 2 - 1, screen.Y / Height * 2 - 1);
        }

        //TODO 需要zBuffer
        public Vector3 ScreenToWorld(Vector2 screen)
        {
            return Vector3.Zero;
        }

        //画直线
        public void DrawLine(Vector2 P0, Vector2 P1, Color C0, Color C1)
        {
            List<float> res;
            List<Color> cols;
            int x0 = (int)P0.X;
            int y0 = (int)P0.Y;
            int x1 = (int)P1.X;
            int y1 = (int)P1.Y;
            if (Math.Abs(x1 - x0) > Math.Abs(y1 - y0))
            {
                if (x0 > x1) { (x0, x1) = (x1, x0); (y0, y1) = (y1, y0); }
                res = Interpolate(x0, y0, x1, y1);
                cols = InterpolateColor(x0, x1, C0, C1);
                for (int i = x0; i <= x1; i++)
                {
                    int y = (int)res[i - x0];
                    if (i >= 0 && i < Bitmap.Width && y >= 0 && y < Bitmap.Height)
                    {
                        Bitmap.SetPixel(i, y, cols[i - x0]);
                    }
                }
            }
            else
            {
                if (y0 > y1) { (x0, x1) = (x1, x0); (y0, y1) = (y1, y0); }
                res = Interpolate(y0, x0, y1, x1);
                cols = InterpolateColor(y0, y1, C0, C1);
                for (int i = y0; i <= y1; i++)
                {
                    int x = (int)res[i - y0];
                    if (i >= 0 && i < Bitmap.Height && x >= 0 && x < Bitmap.Width)
                    {
                        Bitmap.SetPixel(x, i, cols[i - y0]);
                    }
                }
            }

        }

        List<float> Interpolate(float i0, float d0, float i1, float d1)
        {
            List<float> res = new List<float>();
            float d = d0;
            res.Add(d);
            if (i0 == i1) return res;
            float a = (d1 - d0) / (i1 - i0);
            for (float i = i0 + 1; i <= i1; i++)
            {
                d = d + a;
                res.Add(d);
            }
            return res;
        }

        List<Color> InterpolateColor(float a, float b, Color c, Color d)
        {
            List<Color> res = new List<Color>();
            res.Add(c);
            if (a == b) return res;
            float _r = (d.R - c.R) / (b - a);
            float _g = (d.G - c.G) / (b - a);
            float _b = (d.B - c.B) / (b - a);
            for (float i = a + 1; i <= b; i++)
            {
                c = Color.FromArgb(c.A, c.R + (int)_r, c.G + (int)_g, c.B + (int)_b);
                res.Add(c);
            }
            return res;
        }


        public void DrawShadedTriangles(List<Vertex> vertices, List<Triangle> triangles)
        {
            List<Vector2> points = new List<Vector2>();
            List<Color> colors = new List<Color>();
            foreach (var v in vertices)
            {
                points.Add(WorldToScreen(v.Pos));
                colors.Add(v.Color);
            }

            foreach (var t in triangles)
            {
                FillTriangle(points[t.v0], points[t.v1], points[t.v2], colors[t.v0], colors[t.v1], colors[t.v2]);
            }
        }

        //插值填充一个三角形
        public void FillTriangle(Vector2 P0, Vector2 P1, Vector2 P2, Color C0, Color C1, Color C2)
        {
            if (P0.Y > P1.Y)
            {
                (P0, P1) = (P1, P0);
            }
            if (P0.Y > P2.Y)
            {
                (P0, P2) = (P2, P0);
            }
            if (P1.Y > P2.Y)
            {
                (P1, P2) = (P2, P1);
            }
            int y0 = (int)P0.Y;
            int y2 = (int)P2.Y;

            var x01 = Interpolate(P0.Y, P0.X, P1.Y, P1.X);
            var x12 = Interpolate(P1.Y, P1.X, P2.Y, P2.X);
            var x02 = Interpolate(P0.Y, P0.X, P2.Y, P2.X);

            var c01 = InterpolateColor(P0.Y, P1.Y, C0, C1);
            var c12 = InterpolateColor(P1.Y, P2.Y, C1, C2);
            var c02 = InterpolateColor(P0.Y, P2.Y, C0, C2);

            float x1 = x12[0];
            float x2 = x02[x01.Count];

            x01.RemoveAt(x01.Count - 1);
            x01.AddRange(x12);
            c01.RemoveAt(c01.Count - 1);
            c01.AddRange(c12);

            var left = x01;
            var right = x02;
            var leftColor = c01;
            var rightColor = c02;

            if (x1 > x2)
            {
                (left, right) = (right, left);
                (leftColor, rightColor) = (rightColor, leftColor);
            }

            for (int y = y0; y <= y2; y++)
            {
                int i = y - y0;
                DrawLine(new Vector2(left[i], y), new Vector2(right[i], y), leftColor[i], rightColor[i]);
            }
        }


        //绘制世界坐标顶点组成的三角形线框
        public void DrawWireframeTriangles(List<Vector3> vertices, List<Triangle> triangles)
        {
            List<Vector2> points = new List<Vector2>();
            foreach (var v in vertices)
            {
                points.Add(WorldToScreen(v));
            }

            foreach (var t in triangles)
            {
                DrawTriangle(points[t.v0], points[t.v1], points[t.v2]);
            }
        }
        //绘制一个三角形线框
        public void DrawTriangle(Vector2 P0, Vector2 P1, Vector2 P2)
        {
            DrawLine(P0, P1, Color.Black, Color.Black);
            DrawLine(P1, P2, Color.Black, Color.Black);
            DrawLine(P2, P0, Color.Black, Color.Black);
        }
    }
    public class Camera
    {
        public float FieldOfView;//垂直方向的视野夹角
        public float AspactRatio;//长宽比
        public float ZNear;//近平面
        public float ZFar;//远平面

        public Transform Transform;
        public List<Plane> mPlanes = new List<Plane>();


        public GameObject ClipObject(GameObject go)
        {
            foreach (Plane p in mPlanes)
            {
                go = ClipByPlane(p, go);
                if (go == null)
                {
                    return null;
                }
            }
            return go;
        }

        GameObject ClipByPlane(Plane p, GameObject go)
        {
            var mesh = go.Mesh;
            var bs = mesh.GetBoundingSphare();
            float d = SignedDistance(p, bs.Pos);
            if (d > bs.Radius)//视野内
            {
                return go;
            }
            if (d < -bs.Radius)//视野外
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
    }

    public class Light
    {
        public float Intensity;
        public Color LightColor;
    }
    public class AmbientLight : Light
    {
    }

    public class PointLight : Light
    {
        public Vector3 Position;
    }

    public class DirectionLight : Light
    {
        public Vector3 Direction;
    }

    public class Scene
    {
        //光源
        List<Light> mLights = new List<Light>();
        //场景里的物体
        List<GameObject> mObjects = new List<GameObject>();
        //摄像机
        Camera mCamera;
        //屏幕
        Canvas mCanvas;

        public void AddCanvase(Canvas canvas) { mCanvas = canvas; }
        public void AddCamera(Camera cam) { mCamera = cam; }
        public void AddObject(GameObject obj) { mObjects.Add(obj); }
        public void AddLight(Light light) { mLights.Add(light); }

        //渲染物体
        public void Render()
        {
            List<GameObject> clipObjects = new List<GameObject>();
            foreach (GameObject go in mObjects)
            {
                GameObject co = mCamera.ClipObject(go);
                if (co != null)
                {
                    clipObjects.Add(co);
                }
            }

            foreach (GameObject go in clipObjects)
            {
                RenderObject(go);
            }
        }



        void RenderObject(GameObject go)
        {
            if (go.Mesh != null)
            {
                Transform t = go.Transform;
                Transform ct = mCamera.Transform;
                Matrix4x4 model = t.MakeYRotationMatrix() * t.MakeScaleMatrix() * t.MakeTranslationMatrix();
                Matrix4x4 view = ct.MakeTranslationMatrix() * ct.MakeYRotationMatrix();
                List<Vector3> trans = new List<Vector3>();
                foreach (var v in go.Mesh.Verteics)
                {
                    Vector4 v4 = new Vector4(v, 1);
                    Vector4 vt = Multiply(view * model, v4);
                    trans.Add(new Vector3(vt.X, vt.Y, vt.Z));
                }
                mCanvas.DrawWireframeTriangles(trans, go.Mesh.Trangles);
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

    }
}
