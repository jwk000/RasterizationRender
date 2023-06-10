using System.Numerics;

namespace RasterizationRender
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            DoubleBuffered = true;

            ClientSize = new Size(800, 600);
            //TestDrawTriangle();
            //TestDrawObject();
            InitScene();

        }

        Canvas mCanvas = new Canvas(800, 600);
        Scene mScene;

        void InitScene()
        {
            mScene = new Scene();
            var camera = new GameObject();
            camera.AddComponent(mCanvas);
            Transform ct = new Transform();
            ct.Position = new Vector3(0, 0, 0);
            ct.Rotation = new Vector3(0, 0, 0);
            ct.Scale = new Vector3(1, 1, 1);
            camera.AddComponent(ct);
            mScene.AddCamera(camera);
            //以中心为原点，2x2大小立方体
            List<Vector3> vertices = new List<Vector3>()
            {
                new Vector3(1,1,1),
                new Vector3(-1,1,1),
                new Vector3(-1,-1,1),
                new Vector3(1,-1,1),
                new Vector3(1,1,-1),
                new Vector3(-1,1,-1),
                new Vector3(-1,-1,-1),
                new Vector3(1,-1,-1)
            };
            List<Triangle> triangles = new List<Triangle>()
            {
                new Triangle(0,1,2,Color.Red),
                new Triangle(0,2,3,Color.Red),
                new Triangle(4,0,3,Color.Red),
                new Triangle(4,3,7,Color.Red),
                new Triangle(5,4,7,Color.Red),
                new Triangle(5,7,6,Color.Red),
                new Triangle(1,5,6,Color.Red),
                new Triangle(1,6,2,Color.Red),
                new Triangle(4,5,1,Color.Red),
                new Triangle(4,1,0,Color.Red),
                new Triangle(2,6,7,Color.Red),
                new Triangle(2,7,3,Color.Red),

            };
            Mesh m = new Mesh();
            m.vertices = vertices;
            m.triangles = triangles;
            {
                GameObject go = new GameObject();
                go.AddComponent(m);
                Transform t = new Transform();
                t.Position = new Vector3(0, 0, 4);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.AddComponent(t);
                mScene.AddObject(go);
            }
            {
                GameObject go = new GameObject();
                go.AddComponent(m);
                Transform t = new Transform();
                t.Position = new Vector3(-3, 2, 4);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.AddComponent(t);
                mScene.AddObject(go);
            }

            {
                GameObject go = new GameObject();
                go.AddComponent(m);
                Transform t = new Transform();
                t.Position = new Vector3(2, 1, 6);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.AddComponent(t);
                mScene.AddObject(go);
            }

            {
                GameObject go = new GameObject();
                go.AddComponent(m);
                Transform t = new Transform();
                t.Position = new Vector3(-4, -3, 5);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.AddComponent(t);
                mScene.AddObject(go);
            }

            mScene.Render();
        }

        void TestDrawLine()
        {
            mCanvas.DrawLine(new Vector2(100, 100), new Vector2(400, 500), Color.Black);
            mCanvas.DrawLine(new Vector2(100, 500), new Vector2(400, 100), Color.Black);
            Invalidate();

        }

        void TestDrawTriangle()
        {
            var P0 = new Vector2(100, 100);
            var P1 = new Vector2(300, 400);
            var P2 = new Vector2(200, 500);
            mCanvas.FillTriangle(P0, P1, P2, Color.LightGreen);
            mCanvas.DrawTriangle(P0, P1, P2, Color.Black);
            Invalidate();
        }

        void TestDrawObject()
        {
            Vector3 T = new Vector3(-1.5f, 0, 5);
            List<Vector3> vertices = new List<Vector3>()
            {
                new Vector3(1,1,1)+T,
                new Vector3(-1,1,1)+T,
                new Vector3(-1,-1,1)+T,
                new Vector3(1,-1,1)+T,
                new Vector3(1,1,-1)+T,
                new Vector3(-1,1,-1)+T,
                new Vector3(-1,-1,-1)+T,
                new Vector3(1,-1,-1)+T
            };
            List<Triangle> triangles = new List<Triangle>()
            {
                new Triangle(0,1,2,Color.Red),
                new Triangle(0,2,3,Color.Red),
                new Triangle(4,0,3,Color.Red),
                new Triangle(4,3,7,Color.Red),
                new Triangle(5,4,7,Color.Red),
                new Triangle(5,7,6,Color.Red),
                new Triangle(1,5,6,Color.Red),
                new Triangle(1,6,2,Color.Red),
                new Triangle(4,5,1,Color.Red),
                new Triangle(4,1,0,Color.Red),
                new Triangle(2,6,7,Color.Red),
                new Triangle(2,7,3,Color.Red),

            };

            mCanvas.DrawWireframeTriangles(vertices, triangles);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            e.Graphics.DrawImage(mCanvas.mBitmap, 0, 0);
        }
    }
}