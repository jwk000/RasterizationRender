using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RasterizationRender
{
    //包围球
    public struct BoundingSphare
    {
        public float r;
        public Vector3 center;
    }
    public class Mesh :AComponent
    {
        public string name;
        public List<Vector3> vertices; //顶点
        public List<Triangle> triangles;//三角形

        //所有顶点距离原点最近点和最远点为直径
        public BoundingSphare GetBoundingSphare()
        {
            BoundingSphare bs = new BoundingSphare();
            Vector3 near=vertices[0], far=vertices[0];
            foreach(var v in vertices)
            {
                if ( v.Length() < near.Length())
                {
                    near = v;
                }
                if (v.Length() > far.Length())
                {
                    far = v;
                }
            }

            Vector3 d = far - near;
            bs.center = near + d / 2;
            bs.r = d.Length() / 2;
            return bs;
        }
    }



}
