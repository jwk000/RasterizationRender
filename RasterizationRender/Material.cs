using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RasterizationRender
{
    internal class Material:AComponent
    {
        public Color Color = Color.Pink;
        public int Specular = 0;//高光系数
        public float Reflective = 0;//反射系数 0没有反射 1完全反射
        public float Opacity = 1;//不透明度 0完全透明 1完全不透明
        public float Refraction = 1;//折射率 空气的折射率是1 水和玻璃是1.33
    }
}
