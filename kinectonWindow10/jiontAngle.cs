using System;
using System.Collections.Generic;
using System.Linq;
using WindowsPreview.Kinect;

namespace kinectonWindow10
{
   public class jiontAngle
    {
        public jiontAngle()
        {

        }

        public double AngleBetweenJoints(Joint j1, Joint j2, Joint j3)
        {
            double Angulo = 0;
            double shrhX = j1.Position.X - j2.Position.X;
            double shrhY = j1.Position.Y - j2.Position.Y;
            double shrhZ = j1.Position.Z - j2.Position.Z;
            double hsl = vectorNorm(shrhX, shrhY, shrhZ);
            double unrhX = j3.Position.X - j2.Position.X;
            double unrhY = j3.Position.Y - j2.Position.Y;
            double unrhZ = j3.Position.Z - j2.Position.Z;
            double hul = vectorNorm(unrhX, unrhY, unrhZ);
            double mhshu = shrhX * unrhX + shrhY * unrhY + shrhZ * unrhZ;
            double x = mhshu / (hul * hsl);
            if (x != Double.NaN)
            {
                if (-1 <= x && x <= 1)
                {
                    double angleRad = Math.Acos(x);
                    Angulo = angleRad * (180.0 / Math.PI);
                }
                else
                    Angulo = 0;


            }
            else
                Angulo = 0;


            return Angulo;

        }


        /// <summary>
        /// Euclidean norm of 3-component Vector
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        private double vectorNorm(double x, double y, double z)
        {
            return Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2));
        }
        //find the distance of the body
        public double Length(CameraSpacePoint point)
        {
            return Math.Sqrt(
                point.X * point.X +
                point.Y * point.Y +
                point.Z * point.Z
            );
        }
        //outlier detections
        public List<double> StatisticalOutLierAnalysis(List<double> allNumbers)
        {
            List<double> normalNumbers = new List<double>();
            List<double> outLierNumbers = new List<double>();
            double avg = allNumbers.Average();
            double standardDeviation = Math.Sqrt(allNumbers.Average(v => Math.Pow(v - avg, 2)));
            foreach (double number in allNumbers)
            {
                if ((Math.Abs(number - avg)) > (2 * standardDeviation))
                    outLierNumbers.Add(number);
                else
                    normalNumbers.Add(number);
            }

            return normalNumbers;
        }
    }
}
