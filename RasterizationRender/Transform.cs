using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RasterizationRender
{
    public class Transform : AComponent
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
}
