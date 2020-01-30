﻿using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom "using" statements>
using System.Linq;


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


    private void RunScript(bool reset, List<Line> axes, Plane rootFrame, Plane endFrame, Plane target, bool orient, DataTree<Brep> bodies, DataTree<double> range, List<Polyline> structure, int iterations, double error, double angleError, int step, bool animate, int path, List<double> axisControl, ref object Angles, ref object EndPlane, ref object Bodies, ref object TargetPlane, ref object GoalPlane, ref object Solved, ref object TestOut)
    {
        // <Custom code>
        if (targetPlane.Origin != target.Origin || targetPlane.XAxis != target.XAxis || targetPlane.Normal != target.Normal)
        {
            targetPlane = target;
            bot.Restart();
            Print("targetMoved");
            str.Replan();

        }
        if (reset)
        {
            bot.Clear();
            Print("reset");
            bot.InitializeBot(axes, rootFrame, endFrame, bodies, range, error, angleError, iterations);
            str.Replan();
        }
        else
        {
            if (!bot.IsSet)
            {
                Print("initialize");

                bot.InitializeBot(axes, rootFrame, endFrame, bodies, range, error, angleError, iterations);
            }

            if (structure.Count > 0 && !bot.Ran)
            {
                str = new Structure(structure);
                str.GetPaths(bot.RootFrame, targetPlane, 200);
                Print("new Paths Planned");

            }
            if (!bot.GoalSolved && !bot.Ran)
            {
                Print("running");

                if (structure.Count == 0)
                {
                    if (orient)
                    {
                        RhinoApp.WriteLine("Solving with orient");
                        bot.GoalSolved = bot.Solve_IK(targetPlane);
                    }
                    else
                    {
                        RhinoApp.WriteLine("Solving no orient");
                        bot.GoalSolved = bot.Solve_IK(targetPlane.Origin);
                    }
                }
                else
                {
                    if (orient)
                    {
                        RhinoApp.WriteLine("Solving on structure");
                        bot.GoalSolved = bot.Solve_Path(targetPlane, str, false);
                    }
                }
                bot.Ran = true;
                bot.Animate(bot.RobotSteps, 5);
            }

            Robot testBot = new Robot(axes, rootFrame, endFrame, bodies, range, error, angleError, iterations);
            testBot.GoToState(bot.RobotSteps[0]);
            for (int i = 0; i < axisControl.Count; i++)
            {
                testBot.RotateJoint(i, axisControl[i] - testBot.JointAngles[i]);
            }
            testBot.ApplyBodies();

            List<Robot.RobotState> stepsOut = new List<Robot.RobotState>(bot.AnimationFrames);
            if (step >= stepsOut.Count) step = stepsOut.Count - 1;
            if (step < 0) step = 0;
            if (stepsOut.Count > step) bot.GoToState(stepsOut[step]);
            bot.ApplyBodies();

            if (bot.GoalSolved) Print("solve OK");
            Print("totalIterations = {0}", bot.TotalIterations);
            Print("Error = {0}", bot.DistError);
            Print("AngleError = {0}", bot.AngleError);

            if (structure.Count > 0)
            {
                Print("steps = {0}", bot.Steps);
                Print("stepCount = {0}", stepsOut.Count);
            }

            Angles = bot.JointAngles;
            EndPlane = bot.EndFrame;
            Bodies = bot.Bodies;
            TargetPlane = bot.TargetFrame;
            if (str.pathPlanes.Count > 0) GoalPlane = str.pathPlanes[path];
            Solved = stepsOut[step].StateSolved;
            TestOut = testBot.Bodies;

        }
        // </Custom code>
    }

    // <Custom additional code>

    Robot bot = new Robot();
    Structure str = new Structure();
    Plane targetPlane = new Plane();
    public class Robot
    {
        private List<Line> originalAxes = new List<Line>();
        private DataTree<double> jointRange = new DataTree<double>();
        private List<Plane> originalOrientationPlanes = new List<Plane>();
        private List<Plane> reverseOriginalOrientationPlanes = new List<Plane>();
        private List<Plane> orientationPlanes = new List<Plane>();
        private List<Plane> reverseOrientationPlanes = new List<Plane>();
        private DataTree<Brep> originalBodies = new DataTree<Brep>();
        private DataTree<Brep> bodies = new DataTree<Brep>();
        private List<double> jointAngles = new List<double>();
        private List<Line> errorLines = new List<Line> { new Line(), new Line(), new Line() };
        private double distThreshhold;
        private double angleThreshhold;
        private List<RobotState> robotSteps = new List<RobotState>();
        private List<RobotState> robotStates = new List<RobotState>();
        private List<RobotState> animationFrames = new List<RobotState>();
        private Plane targetFrame;
        private Plane goalFrame;
        private int currentStrut;
        private int iterations;
        private int totalIterations;
        private int maxIterations;
        private int steps;
        private int tries;
        private bool stepSolved = false;
        private bool flipped = false;
        private bool goalSolved = false;
        private bool isSet = false;
        private bool ran = false;

        public List<Line> OriginalAxes { get { return originalAxes; } set { originalAxes = value; } }
        public DataTree<double> JointRanges { get { return jointRange; } set { jointRange = value; } }
        public List<Plane> OriginalOrientationPlanes { get { return originalOrientationPlanes; } set { originalOrientationPlanes = value; } }
        public List<Plane> OrientationPlanes { get { return orientationPlanes; } set { orientationPlanes = value; } }
        public DataTree<Brep> OriginalBodies { get { return originalBodies; } set { originalBodies = value; } }
        public DataTree<Brep> Bodies { get { return bodies; } set { bodies = value; } }
        public List<double> JointAngles { get { return jointAngles; } set { jointAngles = value; } }
        public List<Line> ErrorLines { get { return errorLines; } set { errorLines = value; } }
        public double DistError { get { return Math.Abs(EndPoint.X - TargetFrame.Origin.X) + Math.Abs(EndPoint.Y - TargetFrame.Origin.Y) + Math.Abs(EndPoint.Z - TargetFrame.Origin.Z); } }
        public double AngleError { get { return Vector3d.VectorAngle(EndFrame.XAxis, TargetFrame.XAxis) + Vector3d.VectorAngle(EndFrame.YAxis, TargetFrame.YAxis); } }
        public double DistThreshhold { get { return distThreshhold; } set { distThreshhold = value; } }
        public double AngleThreshhold { get { return angleThreshhold; } set { angleThreshhold = value; } }
        public Plane OriginalRootFrame { get { return originalOrientationPlanes[0]; } set { originalOrientationPlanes[0] = value; } }
        public Plane RootFrame { get { return orientationPlanes[0]; } set { orientationPlanes[0] = value; } }
        public Plane OriginalEndFrame { get { return originalOrientationPlanes[originalOrientationPlanes.Count - 1]; } set { originalOrientationPlanes[originalOrientationPlanes.Count - 1] = value; } }
        public Plane EndFrame { get { return orientationPlanes[orientationPlanes.Count - 1]; } set { orientationPlanes[orientationPlanes.Count - 1] = value; } }
        public Point3d EndPoint { get { return EndFrame.Origin; } }
        public Plane TargetFrame { get { return targetFrame; } set { targetFrame = value; } }
        public Plane GoalFrame { get { return goalFrame; } set { goalFrame = value; } }
        public int CurrentStrut { get { return currentStrut; } set { currentStrut = value; } }
        public int Iterations { get { return iterations; } set { iterations = value; } }
        public int TotalIterations { get { return totalIterations; } set { totalIterations = value; } }
        public int MaxIterations { get { return maxIterations; } set { maxIterations = value; } }
        public int Steps { get { return steps; } set { steps = value; } }
        public int Tries { get { return tries; } set { tries = value; } }
        public bool StepSolved { get { return stepSolved; } set { stepSolved = value; } }
        public bool GoalSolved { get { return goalSolved; } set { goalSolved = value; } }
        public bool Flipped { get { return flipped; } set { flipped = value; } }
        public bool IsSet { get { return isSet; } set { isSet = value; } }
        public bool Ran { get { return ran; } set { ran = value; } }
        public List<RobotState> RobotSteps { get { return robotSteps; } set { robotSteps = value; } }
        public List<RobotState> RobotStates { get { return robotStates; } set { robotStates = value; } }
        public List<RobotState> AnimationFrames { get { return animationFrames; } set { animationFrames = value; } }

        public List<object> testOut = new List<object>();
        public Robot() { }
        public Robot(List<Line> axes, Plane rootPlane, Plane endPlane, DataTree<Brep> bodies, DataTree<double> jointRange, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)
        {
            InitializeBot(axes, rootPlane, endPlane, bodies, jointRange, distThreshhold, angleThreshhold, maxIterations);
        }
        public void InitializeBot(List<Line> axes, Plane rootPlane, Plane endPlane, DataTree<Brep> bodies, DataTree<double> jointRange, double distThreshhold = 0.1, double angleThreshhold = 0.01, int maxIterations = 5000)
        {
            OriginalAxes = axes;
            OriginalBodies = bodies;
            JointRanges = jointRange;

            RobotStates.Clear();
            RobotSteps.Clear();
            AnimationFrames.Clear();
            testOut.Clear();
            DistThreshhold = distThreshhold;
            AngleThreshhold = angleThreshhold;
            MaxIterations = maxIterations;
            OriginalOrientationPlanes.Add(rootPlane);

            if (DistThreshhold == 0) DistThreshhold = 0.1;
            if (AngleThreshhold == 0) AngleThreshhold = 0.01;
            if (MaxIterations == 0) MaxIterations = 5000;

            //TargetFrame = EndFrame;

            foreach (Line l in axes)
            {
                OriginalOrientationPlanes.Add(new Plane(l.From, l.Direction));
                JointAngles.Add(0.0);
            }
            OriginalOrientationPlanes.Add(endPlane);

            foreach (Plane p in OriginalOrientationPlanes)
            {
                OrientationPlanes.Add(p);
            }
            GoalSolved = false;
            IsSet = true;
            Ran = false;
            SaveState(ref robotStates, false);
            SaveState(ref robotSteps, false);
            RhinoApp.WriteLine("initializing robot");
        }
        public void Clear()
        {
            Bodies = OriginalBodies;
            OrientationPlanes.Clear();
            OriginalOrientationPlanes.Clear();
            JointAngles.Clear();
            ErrorLines.Clear();
            RobotStates.Clear();
            RobotSteps.Clear();
            AnimationFrames.Clear();
            Iterations = 0;
            TotalIterations = 0;
            Steps = 0;
            Tries = 0;
            CurrentStrut = 0;
            Flipped = false;
            StepSolved = false;
            GoalSolved = false;
            IsSet = false;
            Ran = false;
        }
        public void Restart()
        {
            GoalSolved = false;
            TotalIterations = 0;
            Steps = 0;
            Tries = 0;
            Ran = false;
            if (RobotStates.Count > 0) GoToState(RobotStates[RobotStates.Count - 1]);

        }
        public bool TestDistance(Point3d target, double threshhold = 1, int maxIterations = 20)            // Location Only
        {
            bool success = false;
            while (Iterations < maxIterations && !success)
            {
                stepCCD(target, true);
                if (DistError < threshhold) success = true;
            }
            if (success) SaveState(ref robotStates, StepSolved);
            Iterations = 0;
            return success;
        }
        public bool TestDistance(Structure.Strut strut, double threshhold = 1, int maxIterations = 50)            // Location Only
        {
            TargetFrame = strut.targetPlanes[0];
            bool success = false;
            while (Iterations < maxIterations && !success)
            {
                targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                stepCCD(TargetFrame.Origin, true);
                if (DistError < threshhold) success = true;
            }
            if (success) SaveState(ref robotStates, StepSolved);
            Iterations = 0;
            return success;
        }
        public bool Solve_IK(Point3d target, bool record = false, int earlyBreak = 0)            // Location Only
        {
            bool success = false;
            int stepsStart = RobotStates.Count;

            targetFrame.Origin = target;
            int k = 0;
            while (Iterations < MaxIterations && !success)
            {
                stepCCD(targetFrame.Origin);
                if (record && Iterations == Math.Pow(2, k))
                {
                    SaveState(ref robotStates, StepSolved);
                    k++;
                }
                if (earlyBreak > 0 && Iterations > earlyBreak && DistError > 1) break;
                if (DistError < DistThreshhold) success = true;
            }
            if (!record)
            {
                RobotStates.RemoveRange(stepsStart, RobotStates.Count - stepsStart);
            }
            if (success) SaveState(ref robotStates, StepSolved);
            //NormalizeJointAngles();

            Iterations = 0;
            return success;
        }
        public bool Solve_IK(Plane target, bool record = false, int earlyBreak = 0)            //  with orientation
        {
            bool success = false;
            TargetFrame = target;
            int stepsStart = RobotStates.Count;
            int k = 0;
            while (Iterations < MaxIterations && !success)
            {
                stepCCD(TargetFrame);
                if (record && Iterations == Math.Pow(2, k))
                {
                    SaveState(ref robotStates, false);
                    k++;
                }
                if (earlyBreak > 0 && Iterations > earlyBreak && (DistError > 5 || AngleError > 0.1)) break;

                if (DistError < DistThreshhold && AngleError < AngleThreshhold) success = true;
            }
            if (!record)
            {
                RobotStates.RemoveRange(stepsStart, RobotStates.Count - stepsStart);
            }
            if (success) SaveState(ref robotStates, success);
            //NormalizeJointAngles();

            Iterations = 0;
            return success;
        }
        public bool Solve_IK(Structure.Strut strut, Point3d nextTarget, bool record = false, int earlyBreak = 0)            //  with orientation
        {
            var rot = Transform.Rotation(Math.PI / 2, strut.basePlane.XAxis, strut.basePlane.Origin);
            bool success = false;
            if (TestDistance(strut))
            {
                List<Plane> targets = new List<Plane>();
                foreach (Plane p in strut.targetPlanes) targets.Add(p);
                testOut.Add(strut.walkEnd);
                targets = targets.OrderBy(v => Vector3d.VectorAngle(new Vector3d(strut.walkEnd - nextTarget), v.Normal)).ToList();
                int rotations = 0;
                while (!success && rotations < 4)
                {
                    TargetFrame = targets[rotations];
                    targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                    int k = 0;
                    while (Iterations < MaxIterations && !success)
                    {
                        if (Iterations % 16 == 14) targetFrame.Origin = strut.strutLine.ClosestPoint(EndFrame.Origin, true);
                        stepCCD(TargetFrame);
                        if (record && Iterations == Math.Pow(2, k))
                        {
                            SaveState(ref robotStates, false);
                            k++;
                        }
                        if (earlyBreak > 0 && Iterations == earlyBreak && (DistError > 5 || AngleError > 0.1)) break;
                        if (DistError < DistThreshhold && AngleError < AngleThreshhold) success = true;
                    }
                    rotations++;
                    Iterations = 0;
                }
            }
            if (success) SaveState(ref robotStates, success);
            // NormalizeJointAngles();

            Iterations = 0;
            return success;
        }
        public bool Solve_Path(Plane target, Structure structure, bool record = false)            // on Line Only
        {
            RhinoApp.WriteLine("Solving Path");

            bool success = false;
            GoalFrame = target;

            foreach (List<Structure.Strut> path in structure.paths)
            {

                RhinoApp.WriteLine("pathLength = {0}", path.Count);
                while (CurrentStrut < path.Count && Steps < 50 && Tries < 50)         // try to reach goal following path
                {
                    if (CurrentStrut < path.Count - 2)                                //if not on last strut
                    {
                        StepSolved = Solve_IK(path[CurrentStrut + 1], path[CurrentStrut + 2].walkStart, false, 50);
                        if (StepSolved)         //try to reach next strut
                        {
                            //NormalizeJointAngles();

                            CurrentStrut++;

                            SaveState(ref robotSteps, StepSolved);
                            FlipRobot();

                        }
                        else        //else step on this strut
                        {
                            StepSolved = Solve_IK(path[CurrentStrut], path[CurrentStrut + 1].walkStart, false, 50);
                            if (StepSolved)
                            {
                                //NormalizeJointAngles();

                                SaveState(ref robotSteps, StepSolved);
                                FlipRobot();

                            }
                            else Tries++;
                        }
                    }
                    else if (CurrentStrut == path.Count - 2)                                //if not on last strut
                    {
                        StepSolved = Solve_IK(path[CurrentStrut + 1], GoalFrame.Origin, false, 50);
                        if (StepSolved)         //try to reach next strut
                        {
                            CurrentStrut++;
                            //NormalizeJointAngles();

                            SaveState(ref robotSteps, StepSolved);
                            FlipRobot();

                        }
                        else        //else step on this strut
                        {
                            StepSolved = Solve_IK(path[CurrentStrut], path[CurrentStrut + 1].walkStart, false, 50);
                            if (StepSolved)
                            {
                                //NormalizeJointAngles();

                                SaveState(ref robotSteps, StepSolved);
                                FlipRobot();

                            }
                            else Tries++;
                        }
                    }
                    else                                                        //if on last strut
                    {
                        GoalSolved = Solve_IK(GoalFrame, false);
                        StepSolved = GoalSolved;
                        if (GoalSolved)
                        {
                            //NormalizeJointAngles();

                            SaveState(ref robotSteps, true);
                            break;                                         //if not solved, go to nex path
                        }
                        if (!GoalSolved)
                        {
                            StepSolved = Solve_IK(path[CurrentStrut], goalFrame.Origin, false, 50);        //else step on this strut
                            if (StepSolved)
                            {
                                //NormalizeJointAngles();

                                SaveState(ref robotSteps, StepSolved);
                                FlipRobot();

                            }
                            else Tries++;
                        }
                    }
                }
                if (GoalSolved)
                {
                    RhinoApp.WriteLine("SOLVED!! YAY!");
                    success = GoalSolved;
                    break;                                         //if not solved, go to nex path
                }
                else
                {
                    CurrentStrut = 0;
                    Tries = 0;
                }
            }
            return success;
        }
        private void stepCCD(Point3d target, bool startAtEnd = true)     //Location Only
        {
            if (startAtEnd)
            {
                for (int i = JointAngles.Count - 1; i >= 0; i--)           //for each axis-----ORIGIN  
                {
                    double angle = getAngle(i, target, EndFrame.Origin);
                    RotateJoint(i, angle);
                }
            }
            else
            {
                for (int i = 0; i < JointAngles.Count; i++)           //for each axis-----ORIGIN  
                {
                    double angle = getAngle(i, target, EndFrame.Origin);
                    RotateJoint(i, angle);
                }
            }
            Iterations++;
            TotalIterations++;
        }
        private void stepCCD(Plane target)
        {
            switch (Iterations % 4)
            {
                case 0:           // Orient for each axis-----Z
                    if (AngleError > AngleThreshhold)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.ZAxis, EndFrame.ZAxis);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
                case 1:          // Orient for each axis-----X
                    if (AngleError > AngleThreshhold)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)
                        {
                            double angle = getAngle(i, target.XAxis, EndFrame.XAxis);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
                case 2:
                    if (DistError > DistThreshhold)
                    {
                        for (int i = 0; i < JointAngles.Count; i++)           // Position for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                        }
                    }

                    break;
                case 3:
                    if (DistError > DistThreshhold)
                    {
                        for (int i = JointAngles.Count - 1; i >= 0; i--)           // Position for each axis-----ORIGIN  
                        {
                            double angle = getAngle(i, target.Origin, EndFrame.Origin);
                            RotateJoint(i, angle);
                        }
                    }
                    break;
            }
            Iterations++;
            TotalIterations++;
        }
        private double getAngle(int joint, Point3d target, Point3d end)
        {
            Vector3d jointToEnd = new Vector3d(end - OrientationPlanes[joint + 1].Origin);
            Vector3d jointToGoal = new Vector3d(target - OrientationPlanes[joint + 1].Origin);
            double angle = Vector3d.VectorAngle(jointToEnd, jointToGoal, OrientationPlanes[joint + 1]);
            if (angle < Math.PI * (-2) || angle > Math.PI * (2)) angle = 0;
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRanges.Branch(new GH_Path(joint))[0] != 0 && (JointRanges.Branch(new GH_Path(joint))[0] > (JointAngles[joint] + angle) || (JointAngles[joint] + angle) > JointRanges.Branch(new GH_Path(joint))[1]))
                angle = JointRanges.Branch(new GH_Path(joint))[1] - JointAngles[joint];
            return angle;
        }
        private double getAngle(int joint, Vector3d target, Vector3d end)
        {
            double angle = Vector3d.VectorAngle(end, target, OrientationPlanes[joint + 1]);

            if (angle < Math.PI * (-2) || angle > Math.PI * (2)) angle = 0;
            if (angle > Math.PI) angle = angle - (2 * Math.PI);
            if (JointRanges.Branch(new GH_Path(joint))[0] != 0 && (JointRanges.Branch(new GH_Path(joint))[0] > (JointAngles[joint] + angle) || (JointAngles[joint] + angle) > JointRanges.Branch(new GH_Path(joint))[1]))
                angle = JointRanges.Branch(new GH_Path(joint))[1] - JointAngles[joint];

            angle = angle * (1 - Math.Abs(end * OrientationPlanes[joint + 1].Normal));
            angle = angle * (1 - Math.Abs(target * OrientationPlanes[joint + 1].Normal));

            return angle;
        }
        public void RotateJoint(int joint, double angle)
        {
            var rotate = Transform.Rotation(angle, OrientationPlanes[joint + 1].Normal, OrientationPlanes[joint + 1].Origin);
            if (Flipped)
            {
                for (int i = joint; i < OrientationPlanes.Count - 2; i++)        //move link and children
                {
                    Plane plane = OrientationPlanes[i + 2];
                    plane.Transform(rotate);
                    OrientationPlanes[i + 2] = plane;
                }
            }
            else
            {
                for (int i = joint; i < OrientationPlanes.Count - 1; i++)        //move link and children
                {
                    Plane plane = OrientationPlanes[i + 1];
                    plane.Transform(rotate);
                    OrientationPlanes[i + 1] = plane;
                }
            }
            JointAngles[joint] += angle;
        }
        public void FlipRobot()         // ONLY STUFF FOR IK
        {
            Flipped = !Flipped;
            DataTree<double> rangesTemp = new DataTree<double>();

            for (int i = 0; i < JointAngles.Count; i++)
            {
                foreach (double d in JointRanges.Branch(JointRanges.Branches.Count - 1 - i)) rangesTemp.Add(d, new GH_Path(i));
                JointAngles[i] = -JointAngles[i];
            }
            OrientationPlanes.Reverse();
            StepSolved = false;
            JointAngles.Reverse();
            JointRanges = rangesTemp;
            Steps++;
        }
        public void SaveState(ref List<RobotState> list, bool solved)
        {
            list.Add(new RobotState(RootFrame, JointAngles, OrientationPlanes, TotalIterations, TargetFrame, Flipped, solved));
        }
        private void NormalizeJointAngles()
        {
            for (int i = 0; i < JointAngles.Count; i++)
            {
                RhinoApp.Write("before = {0} ..... ", JointAngles[i]);
                JointAngles[i] = JointAngles[i] % Math.PI;
                RhinoApp.WriteLine("after = {0}", JointAngles[i]);
            }
        }


        public void Animate(List<RobotState> states, int substates)
        {
            Robot animateBot = new Robot(OriginalAxes, OriginalRootFrame, OriginalEndFrame, OriginalBodies, JointRanges);
            for (int i = 0; i < states.Count - 1; i++)
            {
                //animateBot.GoToState(states[i]);
                //if (i > 1) 
                animateBot.SaveState(ref animationFrames, false);
                List<double> stepSizes = new List<double>();
                for (int k = 0; k < animateBot.JointAngles.Count; k++) stepSizes.Add((states[i + 1].StateJointAngles[k] % (2 * Math.PI) - animateBot.JointAngles[k] % (2 * Math.PI)) / substates);
                for (int j = 0; j < substates; j++)
                {
                    for (int k = 0; k < animateBot.JointAngles.Count; k++)
                    {
                        animateBot.RotateJoint(k, stepSizes[k]);
                    }
                    animateBot.SaveState(ref animationFrames, false);
                }
                animateBot.FlipRobot();
                //animateBot.SaveState(ref animationFrames, false);
            }
        }


        public void GoToState(RobotState state)
        {
            Flipped = state.StateFlipped;
            TargetFrame = new Plane(state.StateTargetFrame);
            for (int i = 0; i < state.StateOrientationPlanes.Count; i++) OrientationPlanes[i] = new Plane(state.StateOrientationPlanes[i]);
            for (int i = 0; i < state.StateJointAngles.Count; i++) JointAngles[i] = state.StateJointAngles[i];
        }
        public void ApplyBodies()
        {
            Bodies.Clear();

            if (Flipped)
            {
                for (int i = 0; i < OrientationPlanes.Count - 1; i++)
                {
                    var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[OrientationPlanes.Count - 1 - i]);
                    foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                    {
                        Brep brepTemp = new Brep();
                        brepTemp = b.DuplicateBrep();
                        brepTemp.Transform(reorient);
                        Bodies.Add(brepTemp, new GH_Path(i));
                    }
                }
            }
            else
            {
                for (int i = 0; i < OrientationPlanes.Count - 1; i++)
                {
                    var reorient = Transform.PlaneToPlane(OriginalOrientationPlanes[i], OrientationPlanes[i]);
                    foreach (Brep b in OriginalBodies.Branch(new GH_Path(i)))
                    {
                        Brep brepTemp = new Brep();
                        brepTemp = b.DuplicateBrep();
                        brepTemp.Transform(reorient);
                        Bodies.Add(brepTemp, new GH_Path(i));
                    }
                }
            }
        }


        public class RobotState
        {
            private Plane stateTargetFrame;
            private Plane stateAxes;
            private int stateIterations;
            private bool stateSolved;
            private bool stateFlipped;
            private List<double> stateJointAngles = new List<double>();
            private List<Plane> stateOrientationPlanes = new List<Plane>();

            public Plane StateRootFrame { get { return stateOrientationPlanes[0]; } set { stateOrientationPlanes[0] = value; } }
            public Plane StateTargetFrame { get { return stateTargetFrame; } set { stateTargetFrame = value; } }
            public bool StateSolved { get { return stateSolved; } set { stateSolved = value; } }
            public bool StateFlipped { get { return stateFlipped; } set { stateFlipped = value; } }
            public int StateIterations { get { return stateIterations; } set { stateIterations = value; } }
            public List<double> StateJointAngles { get { return stateJointAngles; } set { stateJointAngles = value; } }
            public List<Plane> StateOrientationPlanes { get { return stateOrientationPlanes; } set { stateOrientationPlanes = value; } }

            public RobotState() { }
            public RobotState(Plane root, List<double> angles, List<Plane> planes, int iterations, Plane target, bool flipped, bool solved)
            {
                StateTargetFrame = new Plane(target);
                StateIterations = iterations;
                StateSolved = solved;
                StateFlipped = flipped;
                foreach (Plane p in planes) StateOrientationPlanes.Add(new Plane(p));
                foreach (double d in angles) StateJointAngles.Add(d);
            }
        }
    }


    public class Structure
    {
        public List<Polyline> structure = new List<Polyline>();
        public List<Strut> struts = new List<Strut>();
        public List<List<Strut>> paths = new List<List<Strut>>();
        public List<List<Plane>> pathPlanes = new List<List<Plane>>();
        public Line first;
        public Line last;
        public List<object> testOut = new List<object>();


        public Structure()
        {

        }
        public Structure(List<Polyline> inputCurves)
        {
            for (int i = 0; i < inputCurves.Count; i++)
            {
                struts.Add(new Strut(inputCurves[i], i));
            }
        }

        public void Replan()
        {
            testOut.Clear();
            paths.Clear();
            pathPlanes.Clear();
        }

        public class Strut
        {
            public Plane basePlane;
            public double length;
            public Line strutLine;
            public int index;
            public int previousStrutIndex;
            public int nextStrutIndex;
            public Point3d walkStart;
            public Point3d walkEnd;
            public List<Plane> targetPlanes = new List<Plane>();

            public Strut()
            {

            }

            public Strut(Strut strutIn)
            {
                basePlane = strutIn.basePlane;
                length = strutIn.length;
                strutLine = strutIn.strutLine;
                index = strutIn.index;
                previousStrutIndex = strutIn.previousStrutIndex;
                nextStrutIndex = strutIn.nextStrutIndex;
                walkStart = strutIn.walkStart;
                walkEnd = strutIn.walkEnd;
                foreach (Plane p in strutIn.targetPlanes) targetPlanes.Add(p);
            }
            public Strut(Polyline inputCurve, int indexIn)
            {
                Point3d origin = inputCurve[1];
                Point3d strutEnd;
                Point3d yPoint;


                if (origin.DistanceTo(inputCurve[0]) > origin.DistanceTo(inputCurve[2]))
                {
                    strutEnd = inputCurve[0];
                    yPoint = inputCurve[2];
                }
                else
                {
                    strutEnd = inputCurve[2];
                    yPoint = inputCurve[0];
                }
                basePlane = new Plane(origin, strutEnd, yPoint);
                length = origin.DistanceTo(strutEnd);
                strutLine = new Line(origin, strutEnd);
                index = indexIn;
                for (int i = 0; i < 4; i++)
                {
                    Plane baseTemp = new Plane(basePlane);
                    var rot = Transform.Rotation(i * Math.PI / 2, basePlane.XAxis, basePlane.Origin);
                    baseTemp.Transform(rot);
                    targetPlanes.Add(baseTemp);
                }
            }
        }

        public void GetPaths(Plane startPlane, Plane endPlane, double maxDistance)
        {
            List<List<Strut>> forwardPaths = new List<List<Strut>>();
            struts = struts.OrderBy(s => s.strutLine.DistanceTo(startPlane.Origin, true)).ToList();
            Strut startStrut = struts[0];
            struts = struts.OrderBy(s => s.strutLine.DistanceTo(endPlane.Origin, true)).ToList();
            Strut targetStrut = struts[0];

            List<List<Strut>> pathsTemp = new List<List<Strut>>();
            pathsTemp.Add(new List<Strut>());
            pathsTemp[0].Add(startStrut);

            int pathsLeft = 0;
            int i = 0;
            while (i < 100 && i < pathsTemp.Count)              // all the paths
            {
                int breakPt = 0;
                bool connected = false;
                bool deadEnd = false;

                List<Strut> thisPath = new List<Strut>(pathsTemp[i]);
                if (startStrut == targetStrut) connected = true;
                while (!connected && !deadEnd && breakPt < 100)            //loops until path is complete keep adding struts until connect, dead end, or timeout
                {
                    LineCurve endStrutLineCurve = new LineCurve(thisPath.Last().strutLine);         // the current end of the path
                    bool reachedOne = false;
                    foreach (Strut s in struts)             //test each other strut
                    {
                        if (!thisPath.Contains(s))        // dont double back or loop
                        {
                            Point3d pointOnEndStrut;
                            Point3d pointOnTestStrut;
                            LineCurve testStrutLineCurve = new LineCurve(s.strutLine);
                            endStrutLineCurve.ClosestPoints(testStrutLineCurve, out pointOnEndStrut, out pointOnTestStrut);

                            if (pointOnEndStrut.DistanceTo(pointOnTestStrut) < maxDistance)         //if strut s is close enough
                            {
                                if (!reachedOne)
                                {
                                    s.walkStart = pointOnTestStrut;
                                    thisPath.Add(new Strut(s));          // if first close strut
                                    thisPath.Last().walkEnd = pointOnEndStrut;
                                }
                                else                                       //if 2nd 3rd...  close strut
                                {
                                    pathsLeft++;
                                    List<Strut> nextPath = new List<Strut>();
                                    foreach (Strut st in thisPath) nextPath.Add(st);
                                    nextPath[nextPath.Count - 1] = s;
                                    pathsTemp.Add(nextPath);
                                }
                                if (s.index == targetStrut.index) connected = true;
                                reachedOne = true;
                            }
                        }
                    }
                    if (!reachedOne) deadEnd = true;
                    breakPt++;
                }
                if (connected) forwardPaths.Add(thisPath);
                for (int l = 0; l < thisPath.Count; l++)
                {
                    for (int k = 0; k < 4; k++)
                    {
                        Plane pTemp = thisPath[l].targetPlanes[k];
                        pTemp.Origin = thisPath[l].walkEnd;
                        thisPath[l].targetPlanes[k] = pTemp;
                    }
                }
                pathsLeft--;
                i++;
                if (pathsLeft < 1) break;
            }
            forwardPaths = forwardPaths.OrderBy(b => b.Count).ToList();

            foreach (List<Strut> l in forwardPaths)                     //for visualization
            {
                List<Plane> planesTemp = new List<Plane>();
                int k = 0;
                foreach (Strut st in l)
                {
                    st.index = k;
                    planesTemp.Add(st.basePlane);
                    k++;
                }
                pathPlanes.Add(planesTemp);
            }
            paths = forwardPaths;
        }



    }
    // </Custom additional code>
}
