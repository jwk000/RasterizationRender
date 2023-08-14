using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace TinyRenderer
{
    class Shader
    {
        public void Vertex(int iface, int nthVert, out Vector4 gl_Position)
        {
            gl_Position = Vector4.Zero;
        }

        public bool Fragment(Vector3 bar, out Color gl_FragColor)
        {
            gl_FragColor = Color.Black;
            return false;
        }
    }
    class Render
    {

    }
}
