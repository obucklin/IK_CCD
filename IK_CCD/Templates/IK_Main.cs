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

        if (or) bot.Solve_IK(t, 1, i);
        else bot.Solve_IK(t.Origin, 1, i);
        bot.ApplyBodies();
        Print("iterations = {0}", bot.Iterations);
        Print("ErrorSq = {0}", bot.ErrorSq);
        Print("Step = {0}", bot.Step);

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
        public double ErrorSq;
        public double ErrorSqOld = 1000000000;
        public Point3d EndPoint;
        public Plane EndFrame;
        public Point3d TargetPoint;
        public Plane TargetPlane;
        public double Step = .5;
        public List<double> Steps = new List<double>();

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
                ErrorSq = Math.Abs(endPoint.X - target.X) + Math.Abs(endPoint.Y - target.Y) + Math.Abs(endPoint.Z - target.Z);
                if (ErrorSq < threshhold)
                {
                    Iterations = i;
                    break;
                }
            }
        }

        public void Solve_IK(Plane target, double threshhold = 0.1, int iterations = 5000)
        {
            Solve_IK(target.Origin);
            TargetPlane = target;
            threshhold = Math.Pow(threshhold, 2);
            for (int i = 0; i < iterations; i++)
            {
                stepJak();                          //Finds Error
                if (ErrorSq > ErrorSqOld * 1.2) Step /= 2;
                ErrorSqOld = ErrorSq;
                Point3d endPoint = new Point3d(Links[Links.Count - 1].To);
                Iterations = i;
                if (ErrorSq < threshhold)
                {
                    break;
                }

            }
        }

        //private List<Vector3d> ErrorVectors()
        //{
        //    //target plane points
        //    Point3d targetCP = TargetPlane.Origin;
        //    Point3d targetX = targetCP + TargetPlane.XAxis * 100;
        //    Point3d targetY = targetCP + TargetPlane.YAxis * 100;

        //    //end effector plane pts
        //    Point3d eeCP = EndFrame.Origin;
        //    Point3d eeX = eeCP + EndFrame.XAxis * 100;
        //    Point3d eeY = eeCP + EndFrame.YAxis * 100;

        //    //error vectors/lines
        //    Vector3d errorCP = targetCP - eeCP;
        //    Vector3d errorX = targetX - eeX;
        //    Vector3d errorY = targetY - eeY;

        //    ErrorSq = Math.Abs(errorCP.X) + Math.Abs(errorCP.Y) + Math.Abs(errorCP.Z) + Math.Abs(errorX.X) + Math.Abs(errorX.Y) + Math.Abs(errorX.Z) + Math.Abs(errorY.X) + Math.Abs(errorY.Y) + Math.Abs(errorY.Z);

        //    ErrorLines[0] = (new Line(eeCP, errorCP));
        //    ErrorLines[1] = (new Line(eeX, errorX));
        //    ErrorLines[2] = (new Line(eeY, errorY));

        //    List<Vector3d> vectorsOut = new List<Vector3d>();
        //    vectorsOut.Add(errorCP);
        //    vectorsOut.Add(errorX);
        //    vectorsOut.Add(errorY);

        //    return vectorsOut;
        //}




        public List<double> axisDelta()
        {
            Point3d targetCP = TargetPlane.Origin;
            Point3d targetX = targetCP + TargetPlane.XAxis * 100;
            Point3d targetY = targetCP + TargetPlane.YAxis * 100;

            //end effector plane pts
            Point3d eeCP = EndFrame.Origin;
            Point3d eeX = eeCP + EndFrame.XAxis * 100;
            Point3d eeY = eeCP + EndFrame.YAxis * 100;

            //error vectors/lines
            Vector3d errorCP = targetCP - eeCP;
            Vector3d errorX = targetX - eeX;
            Vector3d errorY = targetY - eeY;

            ErrorSq = Math.Abs(errorCP.X) + Math.Abs(errorCP.Y) + Math.Abs(errorCP.Z) + Math.Abs(errorX.X) + Math.Abs(errorX.Y) + Math.Abs(errorX.Z) + Math.Abs(errorY.X) + Math.Abs(errorY.Y) + Math.Abs(errorY.Z);
            //RhinoApp.WriteLine("ErrorSq = {0}", ErrorSq);
            //RhinoApp.WriteLine("ErrorSqOld = {0}", ErrorSqOld.ToString());

            //List<Vector3d> vectorsOut = new List<Vector3d>();
            //vectorsOut.Add(errorCP);
            //vectorsOut.Add(errorX);
            //vectorsOut.Add(errorY);
            //List<Vector3d> errorVectors = new List<Vector3d>(ErrorVectors());

            List<double> deltaTemp = new List<double>();

            for (int i = 0; i < Axes.Count; i++)
            {
                //vector axis to end effector points
                Vector3d jointToEndCP = eeCP - Axes[i].ClosestPoint(eeCP, false);
                Vector3d jointToEndX = eeX - Axes[i].ClosestPoint(eeX, false);
                Vector3d jointToEndY = eeY - Axes[i].ClosestPoint(eeY, false);

                //angular vector of rotation about axis[i]
                Vector3d jakobianCP = Vector3d.CrossProduct(Axes[i].Direction, jointToEndCP);
                Vector3d jakobianX = Vector3d.CrossProduct(Axes[i].Direction, jointToEndX);
                Vector3d jakobianY = Vector3d.CrossProduct(Axes[i].Direction, jointToEndY);
                //jakOut.Add(new Line(eeCP, jakobianCP));

                jakobianCP.Unitize();
                jakobianX.Unitize();
                jakobianY.Unitize();

                double dotCP = Vector3d.Multiply(errorCP, jakobianCP);
                double dotX = Vector3d.Multiply(errorX, jakobianX);
                double dotY = Vector3d.Multiply(errorY, jakobianY);

                double thetaCP = 0;
                if (jointToEndCP.Length > 0) thetaCP = Math.Atan(dotCP / jointToEndCP.Length);
                double thetaX = 0;
                if (jointToEndX.Length > 0) thetaX = Math.Atan(dotX / jointToEndX.Length);
                double thetaY = 0;
                if (jointToEndY.Length > 0) thetaY = Math.Atan(dotY / jointToEndY.Length);

                double CPweight = 5;
                double deltaAvg = (thetaCP * CPweight + thetaX + thetaY) / (2 + CPweight);
                deltaAvg *= Step;
                deltaTemp.Add(deltaAvg);
            }

            return deltaTemp;
        }



        private void stepJak()
        {
            Steps = axisDelta();                                // also finds error
            for (int i = Axes.Count - 1; i >= 0; i--)           //for each axis
            {
                if (JointRange.Branch(new GH_Path(i))[0] != 0 && (JointRange.Branch(new GH_Path(i))[0] > (JointAngles[i] + Steps[i]) || (JointAngles[i] + Steps[i]) > JointRange.Branch(new GH_Path(i))[1]))
                    Steps[i] = JointRange.Branch(new GH_Path(i))[1] - JointAngles[i];
                var rot = Transform.Rotation(Steps[i], Axes[i].Direction, Axes[i].From);
                JointAngles[i] += Steps[i];
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
