using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.Windows.Forms;

namespace RasterizationRender
{


    public class Vertex
    {
        public Vector3 Pos;
        public Color Color;
    }

    public struct UV
    {
        public float u;
        public float v;
        public UV(float u, float v) { this.u = u; this.v = v; }
    }
    public struct Triangle
    {
        public int[] index = new int[3];
        public UV[] uv = new UV[3];
        public Vector3[] normal = new Vector3[3];


        public Triangle(int v0, int v1, int v2, UV uv0, UV uv1, UV uv2, Vector3 n0, Vector3 n1, Vector3 n2)
        {
            index[0] = v0;
            index[1] = v1;
            index[2] = v2;
            uv[0] = uv0;
            uv[1] = uv1;
            uv[2] = uv2;
            normal[0] = n0;
            normal[1] = n1;
            normal[2] = n2;

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
        public List<Vector3> Vertexes; //局部顶点
        public List<Triangle> Trangles;//三角形
        public List<Vector3> Normals;//三角形的法线

        public List<Vector3> ViewVertexes;//世界坐标顶点
        //所有顶点距离原点最近点和最远点为直径
        public BoundingSphare GetBoundingSphare()
        {
            Vector3 near = ViewVertexes[0], far = ViewVertexes[0];
            foreach (var v in ViewVertexes)
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
            BoundingSphare bs = new BoundingSphare();
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
            //return Matrix4x4.CreateTranslation(Position);
            return new Matrix4x4(
                1, 0, 0, Position.X,
                0, 1, 0, Position.Y,
                0, 0, 1, Position.Z,
                0, 0, 0, 1);
        }

        public Matrix4x4 MakeXRotationMatrix()
        {
            return Matrix4x4.CreateRotationX(Rotation.X * MathF.PI / 180);
            //float cos = MathF.Cos(Rotation.X * MathF.PI / 180);
            //float sin = MathF.Sin(Rotation.X * MathF.PI / 180);
            //return new Matrix4x4(
            //    1, 0, 0, 0,
            //    0, cos, sin, 0,
            //    0, sin, cos, 0,
            //    0, 0, 0, 1);
        }

        public Matrix4x4 MakeYRotationMatrix()
        {
            return Matrix4x4.CreateRotationY(Rotation.Y * MathF.PI / 180);
            //float cos = MathF.Cos(Rotation.Y * MathF.PI / 180);
            //float sin = MathF.Sin(Rotation.Y * MathF.PI / 180);
            //return new Matrix4x4(
            //    cos, 0, sin, 0,
            //    0, 1, 0, 0,
            //    -sin, 0, cos, 0,
            //    0, 0, 0, 1);
        }

        public Matrix4x4 MakeZRotationMatrix()
        {
            return Matrix4x4.CreateRotationZ(Rotation.Z * MathF.PI / 180);
            //float cos = MathF.Cos(Rotation.Z * MathF.PI / 180);
            //float sin = MathF.Sin(Rotation.Z * MathF.PI / 180);
            //return new Matrix4x4(
            //     cos, -sin, 0, 0,
            //   sin, cos, 0, 0,
            //    0, 0, 1, 0,
            //    0, 0, 0, 1);
        }

        public Matrix4x4 MakeScaleMatrix()
        {
            //return Matrix4x4.CreateScale(Scale);
            return new Matrix4x4(
                Scale.X, 0, 0, 0,
                0, Scale.Y, 0, 0,
                0, 0, Scale.Z, 0,
                0, 0, 0, 1);
        }
    }

    public class Material
    {
        public int Specular = 64;//高光系数
        public Bitmap Texture;//纹理

        public Color GetColor(UV uv)
        {
            int x = (int)(uv.u * (Texture.Width - 1));
            int y = (int)(uv.v * (Texture.Height - 1));
            return Texture.GetPixel(x, y);
        }
    }

    public class GameObject
    {
        public Mesh Mesh;
        public Material Material;
        public Transform Transform;
        public Action<float> OnTick;
        public void Update(float ts)
        {
            OnTick?.Invoke(ts);
        }
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
        public Vector2 ViewPortToScreen(Vector3 vp)
        {
            return NDCToScreen(ViewPortToNDC(vp));
        }
        //近大远小
        public Vector2 ViewPortToNDC(Vector3 vp)
        {
            return new Vector2(vp.X / vp.Z, vp.Y / vp.Z);
        }
        //(-1,1) --> (0,w)
        //(-1,1) --> (0,h)
        public Vector2 NDCToScreen(Vector2 ndc)
        {
            return new Vector2((ndc.X + 1) / 2 * Width, (ndc.Y + 1) / 2 * Height);
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
                cols = ColorInterpolate(x0, x1, C0, C1);
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
                cols = ColorInterpolate(y0, y1, C0, C1);
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

        List<Color> ColorInterpolate(float a, float b, Color c, Color d)
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

        //绘制三角形面
        public void DrawFilledTriangles(List<Vertex> vertices, List<Triangle> triangles)
        {
            List<Vector2> points = new List<Vector2>();
            List<Color> colors = new List<Color>();
            foreach (var v in vertices)
            {
                points.Add(ViewPortToScreen(v.Pos));
                colors.Add(v.Color);
            }

            foreach (var t in triangles)
            {
                FillTriangle(
                    points[t.index[0]], points[t.index[1]], points[t.index[2]],
                    colors[t.index[0]], colors[t.index[1]], colors[t.index[2]]);
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

            var c01 = ColorInterpolate(P0.Y, P1.Y, C0, C1);
            var c12 = ColorInterpolate(P1.Y, P2.Y, C1, C2);
            var c02 = ColorInterpolate(P0.Y, P2.Y, C0, C2);

            float x1 = x12[0];
            float x2 = x02[x01.Count - 1];

            //x01.RemoveAt(x01.Count - 1);
            x01.AddRange(x12);
            //c01.RemoveAt(c01.Count - 1);
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

            for (int y = y0; y < y2; y++)
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
                points.Add(ViewPortToScreen(v));
            }

            foreach (var t in triangles)
            {
                DrawTriangle(points[t.index[0]], points[t.index[1]], points[t.index[2]]);
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
        //左手坐标系（叉乘用左手法则），相机在原点，看向+z方向
        public float FieldOfView;//垂直方向的视野夹角 弧度
        public float AspactRatio;//宽高比 w/h
        public float ZNear;//近平面
        public float ZFar;//远平面
        public Transform Transform;
        public List<Plane> mPlanes = new List<Plane>();
        public int Width;//屏幕宽度
        public int Height;//屏幕高度
        public float ViewW;//视口宽度
        public float ViewH;//视口高度
        public void Init()
        {
            // 计算6个平面
            float n = ZNear;
            float f = ZFar;
            float h = MathF.Tan(FieldOfView / 2) * ZNear;//高度的一半
            float w = h * AspactRatio; //宽度的一半

            ViewW = w * 2;
            ViewH = h * 2;

            Vector3 pTopLeft = new Vector3(-w, h, n);
            Vector3 pTopRight = new Vector3(w, h, n);
            Vector3 pBotLeft = new Vector3(-w, -h, n);
            Vector3 pBotRight = new Vector3(w, -h, n);

            //法线向内
            mPlanes.Add(new Plane(Vector3.Normalize(Vector3.Cross(pTopLeft, pBotLeft)), 0));//left
            mPlanes.Add(new Plane(Vector3.Normalize(Vector3.Cross(pBotRight, pTopRight)), 0));//right
            mPlanes.Add(new Plane(Vector3.Normalize(Vector3.Cross(pTopRight, pTopLeft)), 0));//top
            mPlanes.Add(new Plane(Vector3.Normalize(Vector3.Cross(pBotLeft, pBotRight)), 0));//bottom
            mPlanes.Add(new Plane(new Vector3(0, 0, 1), n));//near
            mPlanes.Add(new Plane(new Vector3(0, 0, -1), -f));//far



        }
        //相机视口投影到屏幕空间
        public Vector2 ViewPort2Canvas(Vector3 v)
        {
            //近大远小
            Vector2 pt = new Vector2(v.X * ZNear / v.Z, v.Y * ZNear / v.Z);
            pt.X = pt.X / ViewW * Width;
            pt.Y = pt.Y / ViewH * Height;

            return pt;
        }

        //屏幕坐标转视口坐标
        public Vector3 Canvas2ViewPort(Vector2 pt, float iz)
        {
            float z = 1 / iz;
            float x = pt.X * z / ZNear;
            float y = pt.Y * z / ZNear;
            Vector3 v = new Vector3(x, y, z);
            v.X = v.X * ViewW / Width;
            v.Y = v.Y * ViewH / Height;
            return v;
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

    class MathHelper
    {
        public static Vector4 Multiply(Matrix4x4 m4, Vector4 v4)
        {
            return new Vector4(
                m4.M11 * v4.X + m4.M12 * v4.Y + m4.M13 * v4.Z + m4.M14 * v4.W,
                m4.M21 * v4.X + m4.M22 * v4.Y + m4.M23 * v4.Z + m4.M24 * v4.W,
                m4.M31 * v4.X + m4.M32 * v4.Y + m4.M33 * v4.Z + m4.M34 * v4.W,
                m4.M41 * v4.X + m4.M42 * v4.Y + m4.M43 * v4.Z + m4.M44 * v4.W
                );
        }

        public static List<float> Interpolate(float i0, float d0, float i1, float d1)
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

        public static List<List<float>> EdgeInterpolate(float y0, float v0, float y1, float v1, float y2, float v2)
        {
            var v01 = Interpolate(y0, v0, y1, v1);
            var v12 = Interpolate(y1, v1, y2, v2);
            var v02 = Interpolate(y0, v0, y2, v2);
            if (v01.Count + v12.Count > v02.Count)
            {
                v02.Add(v02.Last());
            }
            v01.AddRange(v12);

            return new List<List<float>>() { v01, v02 };
        }

        public static float Clamp01(float x)
        {
            if (x < 0) return 0;
            if (x > 1) return 1;
            return x;
        }
    }

    public class Scene
    {
        //光源
        List<Light> mLights = new List<Light>();
        //场景里的物体
        List<GameObject> mObjects = new List<GameObject>();
        //摄像机
        Camera mCamera;

        //深度缓存
        float[] mZBuffer;
        public Bitmap FrameBuffer;

        public void AddCamera(Camera cam) { mCamera = cam; }
        public void AddObject(GameObject obj) { mObjects.Add(obj); }
        public void AddLight(Light light) { mLights.Add(light); }

        public void Update(float ts)
        {
            foreach (var go in mObjects)
            {
                go.Update(ts);
            }
        }

        //渲染物体
        public void Render()
        {
            mZBuffer = new float[mCamera.Width * mCamera.Height];
            FrameBuffer = new Bitmap(mCamera.Width, mCamera.Height);
            foreach (GameObject go in mObjects)
            {
                RenderObject(go);
            }
        }

        void RenderObject(GameObject go)
        {
            //MV变换
            Transform gt = go.Transform;
            Transform ct = mCamera.Transform;
            Matrix4x4 model = gt.MakeTranslationMatrix() * gt.MakeXRotationMatrix() * gt.MakeYRotationMatrix() * gt.MakeScaleMatrix();
            //旋转矩阵转置就是反向旋转
            //平移向量求反构造反向平移矩阵
            Matrix4x4 view = Matrix4x4.Transpose(ct.MakeXRotationMatrix() * ct.MakeYRotationMatrix()) * Matrix4x4.CreateTranslation(-ct.Position);
            Matrix4x4 vm = view * model;
            List<Vector3> vs = new List<Vector3>();
            foreach (var v in go.Mesh.Vertexes)
            {
                Vector4 vt = MathHelper.Multiply(vm, new Vector4(v, 1));
                Vector3 v3 = new Vector3(vt.X, vt.Y, vt.Z);
                vs.Add(v3);
            }
            go.Mesh.ViewVertexes = vs;
            go = ClipObject(go, vm);
            if (go == null)
            {
                return;
            }
            RenderTriangles(vs, go.Mesh.Trangles, go);
        }
        public GameObject ClipObject(GameObject go, Matrix4x4 vm)
        {
            var mesh = go.Mesh;
            var bs = mesh.GetBoundingSphare();
            foreach (Plane p in mCamera.mPlanes)
            {
                if (ClipByPlane(p, bs))
                {
                    return null;
                }
            }
            return go;
        }

        //返回true表示被裁掉了
        bool ClipByPlane(Plane p, BoundingSphare bs)
        {
            float sd = SignedDistance(p, bs.Pos);
            if (sd > bs.Radius)//视野内
            {
                return false;
            }
            if (sd < -bs.Radius)//视野外
            {
                return true;
            }
            //todo 相交
            return false;
        }

        //平面上的点和平面法线点乘得到的投影距离都等于D
        //点v在plane法线的投影距离-plane的距离=相对距离
        float SignedDistance(Plane p, Vector3 v)
        {
            return v.X * p.Normal.X + v.Y * p.Normal.Y + v.Z * p.Normal.Z - p.D;
        }

        void RenderTriangles(List<Vector3> vertics, List<Triangle> triangles, GameObject go)
        {
            //视口顶点投影到屏幕
            List<Vector2> projected = new List<Vector2>();
            foreach (Vector3 v in vertics)
            {
                projected.Add(mCamera.ViewPort2Canvas(v));
            }

            foreach (Triangle t in triangles)
            {
                RenderTriangle(t, vertics, projected, go);
            }
        }

        //绘制一个三角形
        void RenderTriangle(Triangle triangle, List<Vector3> vertics, List<Vector2> projected, GameObject go)
        {
            //三角形背面剔除
            Vector3 tri_center = (vertics[triangle.index[0]] + vertics[triangle.index[1]] + vertics[triangle.index[2]]) / 3;//重心
            Vector3 v01 = vertics[triangle.index[1]] - vertics[triangle.index[0]];
            Vector3 v12 = vertics[triangle.index[2]] - vertics[triangle.index[1]];
            Vector3 tri_normal = Vector3.Cross(v01, v12);
            if (Vector3.Dot(tri_center, tri_normal) > 0)
            {
                return;
            }

            int[] indexes = SortVertexIndexes(triangle.index, projected);

            (int i0, int i1, int i2) = (indexes[0], indexes[1], indexes[2]);
            var (v0, v1, v2) = (vertics[triangle.index[i0]], vertics[triangle.index[i1]], vertics[triangle.index[i2]]);
            var (p0, p1, p2) = (projected[triangle.index[i0]], projected[triangle.index[i1]], projected[triangle.index[i2]]);
            //对x插值
            var x_list = MathHelper.EdgeInterpolate(p0.Y, p0.X, p1.Y, p1.X, p2.Y, p2.X);
            var (x012, x02) = (x_list[0], x_list[1]);
            //对1/z插值
            var iz_list = MathHelper.EdgeInterpolate(p0.Y, 1 / v0.Z, p1.Y, 1 / v1.Z, p2.Y, 1 / v2.Z);
            var (iz012, iz02) = (iz_list[0], iz_list[1]);
            //对uv插值
            var u_list = MathHelper.EdgeInterpolate(p0.Y, triangle.uv[i0].u / v0.Z, p1.Y, triangle.uv[i1].u / v1.Z, p2.Y, triangle.uv[i2].u / v2.Z);
            var (u012, u02) = (u_list[0], u_list[1]);
            var v_list = MathHelper.EdgeInterpolate(p0.Y, triangle.uv[i0].v / v0.Z, p1.Y, triangle.uv[i1].v / v1.Z, p2.Y, triangle.uv[i2].v / v2.Z);
            var (v012, v02) = (v_list[0], v_list[1]);
            //顶点法线变换，法线没有缩放平移，只有model->world->view 两次旋转
            var rotate = Matrix4x4.Transpose(mCamera.Transform.MakeXRotationMatrix() * mCamera.Transform.MakeYRotationMatrix())
                * go.Transform.MakeXRotationMatrix() * go.Transform.MakeYRotationMatrix();
            var n0 = MathHelper.Multiply(rotate, new Vector4(triangle.normal[i0], 1));
            var n1 = MathHelper.Multiply(rotate, new Vector4(triangle.normal[i1], 1));
            var n2 = MathHelper.Multiply(rotate, new Vector4(triangle.normal[i2], 1));

            //phong shading model
            //法线插值
            var nx_list = MathHelper.EdgeInterpolate(p0.Y, n0.X, p1.Y, n1.X, p2.Y, n2.X);
            var (nx012, nx02) = (nx_list[0], nx_list[1]);
            var ny_list = MathHelper.EdgeInterpolate(p0.Y, n0.Y, p1.Y, n1.Y, p2.Y, n2.Y);
            var (ny012, ny02) = (ny_list[0], ny_list[1]);
            var nz_list = MathHelper.EdgeInterpolate(p0.Y, n0.Z, p1.Y, n1.Z, p2.Y, n2.Z);
            var (nz012, nz02) = (nz_list[0], nz_list[1]);

            //判断左和右
            var (xL, xR, izL, izR, nxL, nxR, nyL, nyR, nzL, nzR, uL, uR, vL, vR) =
                (x012, x02, iz012, iz02, nx012, nx02, ny012, ny02, nz012, nz02, u012, u02, v012, v02);
            int m = x02.Count / 2;
            if (x02[m] < x012[m])
            {
                (xL, xR, izL, izR, nxL, nxR, nyL, nyR, nzL, nzR, uL, uR, vL, vR) =
                    (xR, xL, izR, izL, nxR, nxL, nyR, nyL, nzR, nzL, uR, uL, vR, vL);
            }

            //光栅化&着色
            for (int i = 0; i < xL.Count; i++)
            {
                int y = (int)p0.Y + i;
                var izscan = MathHelper.Interpolate(xL[i], izL[i], xR[i], izR[i]);
                var nxscan = MathHelper.Interpolate(xL[i], nxL[i], xR[i], nxR[i]);
                var nyscan = MathHelper.Interpolate(xL[i], nyL[i], xR[i], nyR[i]);
                var nzscan = MathHelper.Interpolate(xL[i], nzL[i], xR[i], nzR[i]);
                var uscan = MathHelper.Interpolate(xL[i], uL[i], xR[i], uR[i]);
                var vscan = MathHelper.Interpolate(xL[i], vL[i], xR[i], vR[i]);

                //渲染像素点x,y
                for (int j = 0; j < izscan.Count; j++)
                {
                    int x = (int)xL[i] + j;
                    float inv_z = izscan[j];
                    if (UpdateDepthBuffer(x, y, inv_z))
                    {
                        Vector3 vertex = mCamera.Canvas2ViewPort(new Vector2(x, y), inv_z);
                        Vector3 normal = Vector3.Normalize(new Vector3(nxscan[j], nyscan[j], nzscan[j]));
                        float intensity = ComputeIllumination(vertex, normal);

                        float u = uscan[j] / izscan[j];
                        float v = vscan[j] / izscan[j];
                        u = MathHelper.Clamp01(u);
                        v = MathHelper.Clamp01(v);
                        Color color = go.Material.GetColor(new UV(u, v));
                        Vector3 vc = new Vector3(color.R, color.G, color.B) * intensity;
                        color = Color.FromArgb((int)vc.X, (int)vc.Y, (int)vc.Z);

                        FrameBuffer.SetPixel(x + mCamera.Width / 2, y + mCamera.Height / 2, color);
                    }
                }
            }
        }

        float ComputeIllumination(Vector3 vertex, Vector3 normal)
        {
            float illumination = 0;
            foreach (Light L in mLights)
            {
                if (L is AmbientLight)
                {
                    illumination += L.Intensity;
                    continue;
                }

                Vector3 vL = Vector3.Zero;

                if (L is DirectionLight)
                {
                    var dL = (DirectionLight)L;
                    var dir = MathHelper.Multiply(Matrix4x4.Transpose(mCamera.Transform.MakeYRotationMatrix()), new Vector4(dL.Direction, 1));
                    vL = Vector3.Normalize(new Vector3(dir.X, dir.Y, dir.Z));
                }
                else if (L is PointLight)
                {
                    var pL = (PointLight)L;
                    var camMatrix = Matrix4x4.Transpose(mCamera.Transform.MakeYRotationMatrix()) * Matrix4x4.CreateTranslation(-mCamera.Transform.Position);
                    var pos = MathHelper.Multiply(camMatrix, new Vector4(pL.Position, 1));
                    Vector3 posL = new Vector3(pos.X, pos.Y, pos.Z);
                    vL = Vector3.Normalize(posL - vertex);
                }


                //diffuse
                float diffuse = L.Intensity * Vector3.Dot(vL, normal);
                if (diffuse > 0)
                {
                    illumination += diffuse;
                }
                //specular
                Vector3 view = -vertex;
                Vector3 half = Vector3.Normalize(vL + view);
                float specular = L.Intensity * MathF.Pow(Vector3.Dot(half, normal), 64);
                if (specular > 0)
                {
                    illumination += specular;
                }
            }
            if (illumination > 1) illumination = 1;
            return illumination;
        }
        bool UpdateDepthBuffer(int x, int y, float iz)
        {
            int ix = (int)mCamera.Width / 2 + x;
            int iy = (int)mCamera.Height / 2 + y;
            if (ix < 0 || ix >= mCamera.Width || iy < 0 || iy >= mCamera.Height)
            {
                return false;
            }
            int i = ix + iy * (int)mCamera.Width;
            if (iz > mZBuffer[i])
            {
                mZBuffer[i] = iz;
                return true;
            }
            return false;
        }
        //顶点按y方向排序
        int[] SortVertexIndexes(int[] vindex, List<Vector2> projected)
        {
            int[] indexes = new[] { 0, 1, 2 };
            if (projected[vindex[indexes[1]]].Y < projected[vindex[indexes[0]]].Y) { int swap = indexes[0]; indexes[0] = indexes[1]; indexes[1] = swap; }
            if (projected[vindex[indexes[2]]].Y < projected[vindex[indexes[0]]].Y) { int swap = indexes[0]; indexes[0] = indexes[2]; indexes[2] = swap; }
            if (projected[vindex[indexes[2]]].Y < projected[vindex[indexes[1]]].Y) { int swap = indexes[1]; indexes[1] = indexes[2]; indexes[2] = swap; }
            return indexes;
        }

    }
}
