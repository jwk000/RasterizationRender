using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RasterizationRender
{
    public class Canvas:AComponent
    {
        public Bitmap mBitmap;
        public int Width { get; private set; }
        public int Height { get; private set; }

        public int ViewWidth { get; private set; }
        public int ViewHeight { get; private set; }
        public int ViewDepth = 1;

        public Canvas(int w, int h)
        {
            Width = w;
            Height = h;
            ViewWidth = w / 200;
            ViewHeight = h / 200;
            mBitmap = new Bitmap(w, h);
        }

        //视口坐标转画布坐标
        public Vector2 ViewportToCanvas(Vector2 V)
        {
            return new Vector2(V.X * Width / ViewWidth + Width/2, V.Y * Height / ViewHeight+Height/2);
        }

        //世界坐标3d顶点投影到画布2d坐标
        public Vector2 ProjectVertex(Vector3 v)
        {
            return ViewportToCanvas(new Vector2(v.X * ViewDepth / v.Z, v.Y * ViewDepth / v.Z));
        }

        public void DrawLine(Vector2 P0, Vector2 P1, Color color)
        {
            List<float> res;
            int x0 = (int)P0.X;
            int y0 = (int)P0.Y;
            int x1 = (int)P1.X;
            int y1 = (int)P1.Y;
            if (Math.Abs(x1 - x0) > Math.Abs(y1 - y0))
            {
                if (x0 > x1) { (x0, x1) = (x1, x0); (y0, y1) = (y1, y0); }
                res = Interpolate(x0, y0, x1, y1);
                for (int i = x0; i <= x1; i++)
                {
                    int y = (int)res[i - x0];
                    if (i >= 0 && i < mBitmap.Width && y >= 0 && y < mBitmap.Height)
                    {
                        mBitmap.SetPixel(i, y, color);
                    }
                }
            }
            else
            {
                if (y0 > y1) { (x0, x1) = (x1, x0); (y0, y1) = (y1, y0); }
                res = Interpolate(y0, x0, y1, x1);
                for (int i = y0; i <= y1; i++)
                {
                    int x = (int)res[i - y0];
                    if(i >= 0 && i < mBitmap.Height && x >= 0 && x < mBitmap.Width)
                    {
                        mBitmap.SetPixel(x, i, color);
                    }
                }
            }

        }

        List<float> Interpolate(float i0, float d0, float i1, float d1)
        {
            List<float> res = new List<float>();
            float a = (d1 - d0) / (i1 - i0);
            float d = d0;
            for (float i = i0; i <= i1; i++)
            {
                res.Add(d);
                d = d + a;
            }
            return res;
        }

        public void DrawTriangle(Vector2 P0, Vector2 P1, Vector2 P2, Color color)
        {
            DrawLine(P0, P1, color);
            DrawLine(P1, P2, color);
            DrawLine(P2, P0, color);
        }

        public void FillTriangle(Vector2 P0, Vector2 P1, Vector2 P2, Color color)
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

            float x1 = x12[0];
            float x2 = x02[x01.Count];

            x01.RemoveAt(x01.Count - 1);
            x01.AddRange(x12);

            var left = x01;
            var right = x02;
            if (x1 > x2)
            {
                (left, right) = (right, left);
            }

            for (int y = y0; y <= y2; y++)
            {
                int i = y - y0;
                float h = i * 1.0f / (y2 - y0);
                DrawLine(new Vector2(left[i], y), new Vector2(right[i], y), Color.FromArgb((int)(color.R * h), (int)(color.G * h), (int)(color.B * h)));
            }
        }


        //绘制线框
        public void DrawWireframeTriangles(List<Vector3> vertices, List<Triangle> triangles)
        {
            List<Vector2> points = new List<Vector2>();
            foreach (var v in vertices)
            {
                points.Add(ProjectVertex(v));
            }

            foreach (var t in triangles)
            {
                DrawTriangle(points[t.v0], points[t.v1], points[t.v2], t.c);
            }
        }
    }

}
