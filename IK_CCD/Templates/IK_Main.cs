using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom "using" statements>



// </Custom "using" statements>


#region padding (this ensures the line number of this file match with those in the code editor of the C# Script component
















#endregion

public partial class MyExternalScript : GH_ScriptInstance
{
    #region Do_not_modify_this_region
    private void Print(string text) { }
    private void Print(string format, params object[] args) { }
    private void Reflect(object obj) { }
    private void Reflect(object obj, string methodName) { }
    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA) { }
    public RhinoDoc RhinoDocument;
    public GH_Document GrasshopperDocument;
    public IGH_Component Component;
    public int Iteration;
    #endregion


    private void RunScript(List<Line> x, Plane t, bool or, DataTree<Brep> g, DataTree<double> r, int i, ref object A, ref object B, ref object C)
    {
        // <Custom code>


        Robot bot = new Robot(x, g, r);

        if (or) bot.Solve_IK(t, 1, 0.1, i);
        else bot.Solve_IK(t.Origin, 1, i);
        bot.ApplyBodies();
        Print("iterations = {0}", bot.Iterations);
        Print("ErrorSq = {0}", bot.DistError);

        A = bot.ErrorLines;
        B = bot.EndFrame;
        C = bot.Bodies;

        // </Custom code>
    }

    // <Custom additional code>

    public class Robot
    {
        public List<Line> Axes = new List<Line>();
        public DataTree<double> JointRange = new DataTree<double>();            
        public List<Plane> OriginalAxisPlanes = new List<Plane>();
        public List<Plane> AxisPlanes = new List<Plane>();
        public DataTree<Brep> OriginalBodies = new DataTree<Brep>();
        public DataTree<Brep> Bodies = new DataTree<Brep>();
        public List<Line> Links = new List<Line>();
        public List<double> JointAngles = new List<double>();
        public List<Line> ErrorLines = new List<Line> { new Line(), new Line(), new Line() };
        public double DistError;        //actually square of dist to shorten calculation time
        public double AngleError;
        public Point3d EndPoint;
        public Plane EndFrame;
        public Point3d TargetPoint;
        public Plane TargetFrame;
        public int Iterations;

        public Robot(List<Line> axes, DataTree<double> jointRange)           //bare skeleton
        {
            Axes = axes;
            JointRange = jointRange;

            for (int i = 0; i < Axes.Count - 1; i++)
            {
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));
                JointAngles.Add(0.0);
            }
            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);
        }

        public Robot(List<Line> axes, DataTree<Brep> bodies, DataTree<double> jointRange)        //with planes for applying geometry
        {
            OriginalBodies = bodies;
            Axes = axes;
            JointRange = jointRange;
            foreach (Line l in Axes)
            {
                OriginalAxisPlanes.Add(new Plane(l.From, l.Direction));
                AxisPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            for (int i = 0; i < Axes.Count - 1; i++)
                Links.Add(new Line(Axes[i].From, Axes[i + 1].From));

            Line endLink = new Line(Links[Links.Count - 1].From, Links[Links.Count - 1].To);
            var move = new Transform(Transform.Translation(new Vector3d(Links[Links.Count - 1].To - Links[Links.Count - 1].From)));
            endLink.Transform(move);
            Links.Add(endLink);
            EndFrame = new Plane(endLink.To, endLink.Direction);

        }

        private void stepCCD()     //Location Only
        {
            for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis
            {
                Vector3d jointToEnd = new Vector3d(Links[Links.Count - 1].To - Axes[i].From);
                Vector3d jointToGoal = new Vector3d(TargetPoint - Axes[i].From);

                double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);

                if (angle > Math.PI) angle = angle - (2 * Math.PI);
                if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                    angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
                var rot = Transform.Rotation(angle, Axes[i].Direction, Axes[i].From);
                JointAngles[i] += angle;
                EndFrame.Transform(rot);

                for (int j = i; j < Axes.Count; j++)        //move link and children
                {
                    Line axis = Axes[j];
                    Line link = Links[j];
                    Plane plane = AxisPlanes[j];
                    axis.Transform(rot);
                    link.Transform(rot);
                    plane.Transform(rot);
                    Axes[j] = axis;
                    Links[j] = link;
                    AxisPlanes[j] = plane;
                }
            }
        }

        public void Solve_IK(Point3d target, double threshhold = 0.1, int iterations = 5000)            // Location Only
        {
            TargetPoint = target;
            threshhold = Math.Pow(threshhold, 2);
            for (int i = 0; i < iterations; i++)
            {
                stepCCD();
                Point3d endPoint = new Point3d(Links[Links.Count - 1].To);
                DistError = Math.Abs(endPoint.X - target.X) + Math.Abs(endPoint.Y - target.Y) + Math.Abs(endPoint.Z - target.Z);
                if (DistError < threshhold)
                {
                    Iterations = i;
                    break;
                }
            }
        }


        public void Solve_IK(Plane target, double distThreshhold = 0.1, double angleThreshhold = 0.01, int iterations = 5000)            // Location Only
        {
            TargetFrame = target;
            distThreshhold = Math.Pow(distThreshhold, 2);
            for (int i = 0; i < iterations; i++)
            {
                stepCCD_Orient();
                Point3d endPoint = new Point3d(Links[Links.Count - 1].To);
                DistError = Math.Abs(EndFrame.Origin.X - target.Origin.X) + Math.Abs(EndFrame.Origin.Y - target.Origin.Y) + Math.Abs(EndFrame.Origin.Z - target.Origin.Z);
                AngleError = Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis);
                Iterations = i;
                if (DistError < distThreshhold && AngleError < angleThreshhold)
                    break;
            }
        }
        private void stepCCD_Orient()
        {
            for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----X
            {
                Point3d targetX = TargetFrame.Origin + TargetFrame.XAxis * 100;
                Point3d eeX = EndFrame.Origin + EndFrame.XAxis * 100;
                var rot = Rotate(i, targetX, eeX);
                EndFrame.Transform(rot);

                for (int j = i; j < Axes.Count; j++)        //move link and children
                {
                    rotateAxes(j, rot);
                }
            }

            for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----Y
            {
                Point3d targetY = TargetFrame.Origin + TargetFrame.YAxis * 100;
                Point3d eeY = EndFrame.Origin + EndFrame.YAxis * 100;
                var rot = Rotate(i, targetY, eeY);
                EndFrame.Transform(rot);
                for (int j = i; j < Axes.Count; j++)        //move link and children
                {
                    rotateAxes(j, rot);
                }
            }

            //for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN
            //{
            //    var rot = Rotate(i, TargetFrame.Origin, EndFrame.Origin);
            //    EndFrame.Transform(rot);
            //    for (int j = i; j < Axes.Count; j++)        //move link and children
            //    {
            //        rotateAxes(j, rot);
            //    }
            //}

            for (int i = 0; i <= Axes.Count - 1; i++)           //for each axis-----ORIGIN
            {
                var rot = Rotate(i, TargetFrame.Origin, EndFrame.Origin);
                EndFrame.Transform(rot);
                for (int j = i; j < Axes.Count; j++)        //move link and children
                {
                    rotateAxes(j, rot);
                }
            }
        }

        private Transform Rotate(int i, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - Axes[i].From);
            Vector3d jointToGoal = new Vector3d(target - Axes[i].From);

            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, AxisPlanes[i]);

            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + angle) || (JointAngles[i] + angle) > JointRange.Branch(new GH_Path(i))[1]))
                angle = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
            var rot = Transform.Rotation(angle , Axes[i].Direction, Axes[i].From);
            JointAngles[i] += angle;
            return rot;
        }
        private void rotateAxes(int i, Transform rot)
        {
            Line axis = Axes[i];
            Line link = Links[i];
            Plane plane = AxisPlanes[i];
            axis.Transform(rot);
            link.Transform(rot);
            plane.Transform(rot);
            Axes[i] = axis;
            Links[i] = link;
            AxisPlanes[i] = plane;
        }

        public void ApplyBodies()
        {
            for (int i = 0; i < AxisPlanes.Count; i++)
            {
                var reorient = Transform.PlaneToPlane(OriginalAxisPlanes[i], AxisPlanes[i]);
                foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                {
                    Brep brepTemp = new Brep();
                    brepTemp = b;
                    brepTemp.Transform(reorient);
                    Bodies.Add(brepTemp, new GH_Path(i));
                }
            }
        }
    }



    // </Custom additional code>
}
