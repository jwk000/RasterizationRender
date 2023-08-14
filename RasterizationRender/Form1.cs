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
                camera.ZNear = 1;
                camera.ZFar = 100;
                camera.FieldOfView = MathF.PI / 2;
                camera.AspactRatio = 16.0f / 9.0f;

                Transform ct = new Transform();
                ct.Position = new Vector3(0, 0, 0);
                ct.Rotation = new Vector3(0, 0, 0);
                ct.Scale = new Vector3(1, 1, 1);
                camera.Transform = ct;

                camera.Init();
                mScene.AddCamera(camera);
            }
            {
                DirectionLight directionLight = new DirectionLight();
                directionLight.Direction = new Vector3(0, -1, 1);
                directionLight.LightColor = Color.White;
                directionLight.Intensity = 0.4f;
                mScene.AddLight(directionLight);
            }
            {
                PointLight pointLight = new PointLight();
                pointLight.Position = new Vector3(-3, 2, -10);
                pointLight.LightColor = Color.Yellow;
                pointLight.Intensity = 0.2f;
                mScene.AddLight(pointLight);
            }
            {
                AmbientLight ambientLight = new AmbientLight();
                ambientLight.LightColor = Color.White;
                ambientLight.Intensity = 0.2f;
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
            UV uv0 = new UV(0, 0);
            UV uv1 = new UV(1, 0);
            UV uv2 = new UV(1, 1);
            UV uv3 = new UV(0, 1);


            Vector3 back = new Vector3(0, 0, 1);//back
            Vector3 right = new Vector3(1, 0, 0);//right
            Vector3 front = new Vector3(0, 0, -1);//front
            Vector3 left = new Vector3(-1, 0, 0);//left
            Vector3 up = new Vector3(0, 1, 0);//up
            Vector3 down = new Vector3(0, -1, 0);//down

            List<Triangle> triangles = new List<Triangle>()
            {
                new Triangle(0,1,2,uv0,uv1,uv2,back,back,back),
                new Triangle(0, 2, 3, uv0, uv2, uv3, back, back, back),
                new Triangle(4, 0, 3, uv0, uv1, uv2,right,right,right),
                new Triangle(4, 3, 7, uv0, uv2, uv3,right,right,right),
                new Triangle(5, 4, 7, uv0, uv1, uv2,front,front,front),
                new Triangle(5, 7, 6, uv0, uv2, uv3,front,front,front),
                new Triangle(1, 5, 6, uv0, uv1, uv2,left,left,left),
                new Triangle(1, 6, 2, uv0, uv2, uv3,left,left,left),
                new Triangle(1, 0, 5, uv0, uv1, uv2,up,up,up),
                new Triangle(5, 0, 4, uv3, uv2, uv0,up,up,up),
                new Triangle(2, 6, 7, uv0, uv1, uv2,down,down,down),
                new Triangle(2, 7, 3, uv0, uv2, uv3,down,down,down),

            };

            Bitmap Texture = new Bitmap("./data/crate-texture.jpg");

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
                Material material = new Material();
                go.Material = material;
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
                Material material = new Material();
                material.Texture = Texture;
                go.Material = material;
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
                Material material = new Material();
                go.Material = material;
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
                Material material = new Material();
                go.Material = material;
                mScene.AddObject(go);
            }

            mScene.Render();
            Invalidate();
        }


    }
}