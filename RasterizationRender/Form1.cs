using System.Numerics;

namespace RasterizationRender
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            DoubleBuffered = true;
            ClientSize = new Size(1024, 768);
            mCanvas = new Canvas(1024, 768);
            InitScene();

        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            e.Graphics.DrawImage(mCanvas.Bitmap, 0, 0);
        }


        Canvas mCanvas;
        Scene mScene;

        void InitScene()
        {
            mScene = new Scene();
            mScene.AddCanvase(mCanvas);

            {
                var camera = new Camera();
                Transform ct = new Transform();
                ct.Position = new Vector3(0, 0, 0);
                ct.Rotation = new Vector3(0, 0, 0);
                ct.Scale = new Vector3(1, 1, 1);
                camera.Transform = ct;
                mScene.AddCamera(camera);
            }

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
                new Triangle(0,1,2),
                new Triangle(0,2,3),
                new Triangle(4,0,3),
                new Triangle(4,3,7),
                new Triangle(5,4,7),
                new Triangle(5,7,6),
                new Triangle(1,5,6),
                new Triangle(1,6,2),
                new Triangle(4,5,1),
                new Triangle(4,1,0),
                new Triangle(2,6,7),
                new Triangle(2,7,3),

            };
            Mesh m = new Mesh();
            m.Verteics = vertices;
            m.Trangles = triangles;
            {
                GameObject go = new GameObject();
                go.Mesh = m;
                Transform t = new Transform();
                t.Position = new Vector3(0, 0, 4);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.Transform = t;
                mScene.AddObject(go);
            }
            {
                GameObject go = new GameObject();
                go.Mesh = m;
                Transform t = new Transform();
                t.Position = new Vector3(-3, 2, 4);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.Transform = t;
                mScene.AddObject(go);
            }

            {
                GameObject go = new GameObject();
                go.Mesh = m;
                Transform t = new Transform();
                t.Position = new Vector3(2, 1, 6);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.Transform = t;
                mScene.AddObject(go);
            }

            {
                GameObject go = new GameObject();
                go.Mesh = m;
                Transform t = new Transform();
                t.Position = new Vector3(-4, -3, 5);
                t.Rotation = new Vector3(0, 0, 0);
                t.Scale = new Vector3(1, 1, 1);
                go.Transform = t;
                mScene.AddObject(go);
            }

            mScene.Render();
            Invalidate();
        }


    }
}