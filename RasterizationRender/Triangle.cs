using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RasterizationRender
{
    public struct Triangle
    {
        public int v0;
        public int v1;
        public int v2;
        public Color c;

        public Triangle(int v0, int v1, int v2, Color c)
        {
            this.v0 = v0;
            this.v1 = v1;
            this.v2 = v2;
            this.c = c;
        }
    }
}
