using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        /*
        * Author: Wanderer

        Version: 2.4.2
        - Slight improvement of wheel adding algorithm.

        */
        class DriverAssistingSystem
        {
            #region mdk preserve
            // General settings.
            double _MaxForwardAccel = 10;// Maximum forward acceleration in m/s^2.
            double _MaxBackwardAccel = 10;// Maximum backward acceleration in m/s^2.
            double _MaxPowerConsumption = 1;// Maximum allowed power output to wheels.
            double _NaturalPitchDegree = 0;// If vehicle buld with chassis incline in mind,put this incline here.
            double _SafeFallVelocity = 15;// Vertical velocity which FDS will try to achive near ground level.
            double _LeanToSurfaceDistance = 15;// Distance at which script will align vehicle to surface curve. Downward camera required.
            readonly string _StopLightsGroupName = "Stop lights";// Group name of stop/back/break lights which script will search.
            readonly string _TurnLightsGroupName = "Turn lights";// Group name of stop/back/break lights which script will search.
            readonly string _SubgridWheelsGroupName = "Subgrid Wheels";// Group of wheels on subgrid. These wheels control will be overrien.
            readonly string _BackupBeeperName = "Back-up beeper";// Sound block(s) that will triggered to play when driving backward.
            
            HandbrakeMode _HandbrakeMode = HandbrakeMode.Auto;// Auto handbrakes mode:
                                                              // HandbrakeMode.Semi-turn hand brakes ON when no one control vehicle,don't turn OFF when someone get control;
                                                              // HandbrakeMode.Auto-turn hand brakes ON when no one control vehicle,turn OFF when someone get control;
                                                              // HandbrakeMode.Off -don't turn hand brakes ON when no one control vehicle and don't turn OFF when someone get control.

            readonly AckermannFocalPointRef _AckermannFocalPoint = AckermannFocalPointRef.CoM;// Define main turn axle for all wheels
                                                                                     // AckermannFocalPointRef.CoM-main turn axel will be vector from vehicle CoM to Ackermann focal point. Most stable.
                                                                                     // AckermannFocalPointRef.RC-main turn axel will be vector from RC block position to Ackermann focal point. RC block should be between left and right wheels.
                                                                                     // AckermannFocalPointRef.BSphere-main turn axel will be vector from bounding sphere centre to Ackermann focal point.
            bool _UseJumpJets = true;// Engage upward thrusters when Spacebar pressed.
            bool _UseAdaptiveSteering = true;// Reduce steering angle when driving at high speed.
            bool _UseAckermannSteering = true;// Use Ackermann steering scheme.
            bool _UseDASAirShock = true; // Use air shock system for suspensions provided by DAS instead of vanilla one.
            bool _UseHillDescentControl = true;// Use use soft brakes if lateral velocity exceeds speed limit.

            bool _UseGSA = true;// Override gyros or not.
            bool _UseTCS = true;// Use all stuff tied to suspensions.
            bool _UseFDS = true;// Override upward thrusters or not.
            bool _UseSDO = true;// Override other thrusters?

            bool _UseLights = true;// Handle stop and turn lights or not.

            // Suspension settings.
            double _FrwrdSpeedLimitKPH = 180;// Suspension wheels speed limit when driving worward.
            double _BckwrdSpeedLimitKPH = 80;// Suspension wheels speed limit when driving backward.
            double _MaxSteerAngleDegree = 25;// Suspension wheel maximum steering angle.
            double _SusHeightOffset = -1.5;// Suspension heght offset that will be setted by script. Can be changed.
            double _SusStrenght = -1;// Suspension strenght that will be setted by script. Can be changed. Negative value enables auto calculation.
            double _SusValueChangeRate = 15;// Rate at wich script will adjuct height offset and strenght related to previous value. Mesured in game ticks.

            // Autopilot settings.
            readonly string _AutopilotTimerName = "Timer Block DAS Waypoint";// This timer block will be triggered at every waypoint.
            double _AutopilotAccuracy = 0.75;// How close to waypoint vehicle should be to consider it's reached. Higher-closer.
            double _AutopilotWait = 5;// Wait this time in sec when waypoint reached.
            double _AutopilotSpeedLimitKMP = 60;// Will drive at this speed.

            //-------------------------------------------------------------------
            // Script stuff. Modifying anything below will void your warranty.
            //-------------------------------------------------------------------

            enum HandbrakeMode : byte { Semi, Auto, Off, END_OF_ENUM }
            enum AckermannFocalPointRef : byte { CoM, RC, BSphere }

            #endregion

            class RuntimeProfiler
            {
                readonly Program Parent;
                double RunTimeAvrEMA;
                readonly double EMA_A;
                int CycleNum;
                int Counter;
                readonly StringBuilder Str;
                readonly IMyTextSurface MyLCD;
                IEnumerator<bool> StMachine;
                public int SkipCycles;
                public string Caption;
                public string Memo;

                public RuntimeProfiler(Program parent, int skipCycles = 0, int skipOffset = 0, double emaA = 0.003)
                {
                    Parent = parent;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    EMA_A = emaA;
                    Str = new StringBuilder();
                    if (Parent.Me.CustomData == "")
                        MyLCD = Parent.Me.GetSurface(0);
                    StMachine = UpdateIterator();
                }

                public void Update()
                {
                    if (!StMachine.MoveNext())
                        StMachine = UpdateIterator();
                }

                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                    {
                        RunTimeAvrEMA = EMA_A * Parent.Runtime.LastRunTimeMs + (1 - EMA_A) * RunTimeAvrEMA;
                        yield return true;
                    }
                    CycleNum = 0;
                    Str.Clear();
                    Str.Append(Caption);
                    ++Counter;
                    switch (Counter % 4)
                    {
                        case 0: Str.Append("--"); break;
                        case 1: Str.Append("\\"); break;
                        case 2: Str.Append(" |"); break;
                        case 3: Str.Append("/"); break;
                    }
                    Str.AppendFormat("\n{0}\n", Memo);
                    RunTimeAvrEMA = EMA_A * Parent.Runtime.LastRunTimeMs + (1 - EMA_A) * RunTimeAvrEMA;
                    Str.AppendFormat("Instructions used: {0:G}/{1:G}\nAverage(EMA) run time: {2:F3} ms\nLast run time: {3:F3} ms\n{4}",
                    Parent.Runtime.CurrentInstructionCount,
                    Parent.Runtime.MaxInstructionCount,
                    RunTimeAvrEMA,
                    Parent.Runtime.LastRunTimeMs,
                    Counter % 2 == 0 ? "_" : "");
                    Parent.Echo(Str.ToString());
                    MyLCD?.WriteText(Str);
                }
            }

            class PID
            {
                public double Kp { get; set; }
                public double Ki { get; set; }
                public double Kd { get; set; }
                public double Signal { get; set; }
                double ValuePrev;
                double Integral;
                double TimePrev;

                public PID(double kp = 1, double ki = 1, double kd = 1)
                {
                    Kp = kp;
                    Ki = ki;
                    Kd = kd;
                }

                public void Reset()
                {
                    ValuePrev = 0;
                    Integral = 0;
                    TimePrev = 0;
                    Signal = 0;
                }

                public double GetSignal(double value, double time = 0)
                {
                    if (Ki != 0)
                        Integral += value * 0.001;
                    double dt = time == 0 ? 1 : time - TimePrev;
                    Signal = Kp * value + Ki * Integral * dt + Kd * (value - ValuePrev) / dt;
                    ValuePrev = value;
                    TimePrev = time;
                    return Signal;
                }
            }

            class Utils
            {
                static public bool IsIgnore(IMyTerminalBlock block)
                {
                    return block.CustomData.Equals("DAS_IGNORE", StringComparison.OrdinalIgnoreCase);
                }
            }

            class StringQueue
            {
                readonly List<string> StringsList;
                readonly StringBuilder String;
                public int MaxCount { get; set; }

                public StringQueue(int maxCount = 0)
                {
                    StringsList = new List<string>();
                    String = new StringBuilder();
                    MaxCount = maxCount;
                }

                public void Append(string str)
                {
                    StringsList.Add(str);
                    if (MaxCount > 0 && StringsList.Count > MaxCount)
                        StringsList.RemoveAt(0);
                }
                public void Remove(int count = 1)
                {
                    StringsList.RemoveRange(0, Math.Min(StringsList.Count, count));
                }
                public StringBuilder GetString()
                {
                    String.Clear();
                    foreach (var item in StringsList)
                        String.AppendLine(item);
                    return String;
                }
            }

            class BaseClass
            {
                protected Program Parent;
                public IMyRemoteControl RC;
                public bool Ready { get { return RC != null; } }
                public BaseClass(Program parent) { Parent = parent; }
                public IMyRemoteControl ForceDetectRemoteControl()
                {
                    RC = null;
                    List<IMyRemoteControl> remcomarr = new List<IMyRemoteControl>();
                    Parent.GridTerminalSystem.GetBlocksOfType(remcomarr,
                    block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                    if (remcomarr.Count > 0)
                        RC = remcomarr[0];
                    return RC;
                }
            }

            interface IInputData
            {
                float WS { get; }
                float AD { get; }
                float CSpacebar { get; }
                float QE { get; }
                bool Ready { get; }
                bool Manning { get; }
            }
            class UserInputWatcher : IInputData
            {
                readonly Program Parent;
                readonly List<IMyShipController> Controllers;
                int CycleNum;
                int Indx;
                IEnumerator<bool> StMachine;

                public float WS { get; set; }
                public float AD { get; set; }
                public float CSpacebar { get; set; }
                public float QE { get; set; }
                public bool Ready { get; set; }
                public bool Manning { get; set; }
                public int SkipCycles;

                public UserInputWatcher(Program parent, int skipCycles = 0, int skipOffset = 0)
                {
                    Parent = parent;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    Controllers = new List<IMyShipController>();
                    StMachine = UpdateIterator();
                }
                public void DetectFuncBlocks()
                {
                    Controllers.Clear();
                    Parent.GridTerminalSystem.GetBlocksOfType(Controllers,
                    block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                    Ready = Controllers.Count > 0;
                    Indx = 0;
                }
                public void Reset()
                {
                    WS = AD = CSpacebar = QE = 0;
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;
                    Manning = false;
                    Vector3 input = Vector3.Zero;
                    QE = 0;
                    for (int i = 0; i < Controllers.Count; ++i)
                    {
                        if (Controllers[Indx].IsUnderControl)
                        {
                            input = Controllers[Indx].MoveIndicator;
                            QE = Controllers[Indx].RollIndicator;
                            Manning = true;
                            break;
                        }
                        Indx = (Indx + 1) % Controllers.Count;
                    }
                    WS = input.Z;
                    AD = input.X;
                    CSpacebar = input.Y;
                }
            }
            class AutopilotDriver : BaseClass, IInputData
            {
                int CycleNum;
                IEnumerator<bool> StMachine;
                bool EnableState;
                readonly string TimerName;
                IMyTimerBlock Timer;

                public enum EDriveMode : byte { OneWay, Patrol, Cicrle, END_OF_ENUM }

                public int SkipCycles;
                public int WaypointIndx;
                public int WaypointSelector = 1;
                public float WS { get; set; }
                public float AD { get; set; }
                public float CSpacebar { get; set; }
                public float QE { get; set; }
                public bool Manning { get; set; }
                public bool Busy { get; set; }
                public MyWaypointInfo CurrentWaypoint { get; set; }
                public MyWaypointInfo FirstWaypoint { get; set; }
                public MyWaypointInfo LastWaypoint { get; set; }
                public double Time { get; set; }
                public EDriveMode DriveMode;
                public bool Enabled;
                public double dT = 0.016;
                public double Accuracy;
                public double WaitTimeSec;

                public AutopilotDriver(Program parent, string timerName, int skipCycles = 0, int skipOffset = 0) : base(parent)
                {
                    TimerName = timerName;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                }
                public void ResetWaypointIndex(bool reset = true)
                {
                    if (reset)
                    {
                        WaypointIndx = 0;
                        WaypointSelector = 1;
                    }
                }
                public void InverseRoute()
                {
                    List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
                    RC.GetWaypointInfo(waypoints);
                    RC.ClearWaypoints();
                    for (int i = waypoints.Count - 1; i >= 0; i--)
                        RC.AddWaypoint(waypoints[i]);
                    if (WaypointIndx > 0)
                        WaypointIndx = waypoints.Count - WaypointIndx;
                    StMachine = UpdateIterator();
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    Timer = Parent.GridTerminalSystem.GetBlockWithName(TimerName) as IMyTimerBlock;
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!StMachine.MoveNext())
                                StMachine = UpdateIterator();
                        }
                        else if (EnableState)
                        {
                            Manning = Busy = false;
                            WS = AD = 0;
                            Time = 0;
                            StMachine = UpdateIterator();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateIterator()
                {
                    List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
                    RC.GetWaypointInfo(waypoints);
                    if (waypoints.Count == 0) yield return false;
                    FirstWaypoint = waypoints[0];
                    LastWaypoint = waypoints[waypoints.Count - 1];
                    Manning = true;
                    WS = AD = 0;
                    Busy = true;
                    WaypointIndx = MathHelper.Clamp(WaypointIndx, 0, waypoints.Count - 1);
                    Time = 0;
                    while (Busy)
                    {
                        for (; CycleNum < SkipCycles; ++CycleNum)
                        {
                            Time += dT;
                            yield return true;
                        }
                        CycleNum = 0;
                        CurrentWaypoint = waypoints[WaypointIndx];
                        Vector3D targetVector = CurrentWaypoint.Coords - RC.CubeGrid.WorldVolume.Center;
                        double boundSphereRadi = RC.CubeGrid.WorldVolume.Radius;
                        double temp = boundSphereRadi / Accuracy;
                        if (targetVector.LengthSquared() > temp * temp)
                        {
                            Time = 0;
                            RC.HandBrake = false;
                            float dir = (float)RC.WorldMatrix.Backward.Dot(targetVector) + 0.3f;
                            WS = MathHelper.Clamp(dir + (dir > 0 ? 0.1f : -0.1f), -1, 1);
                            AD = (float)MathHelper.Clamp(RC.WorldMatrix.Right.Dot(Vector3D.Normalize(targetVector)) * Math.Sign(-WS) * 2, -1, 1);
                            yield return true;
                        }
                        else
                        {
                            RC.HandBrake = WaitTimeSec > 0;
                            Time += dT;
                            WS = AD = 0;
                            Timer?.Trigger();
                            if (Time < WaitTimeSec)
                                yield return true;
                            else
                            {
                                Time = 0;
                                WaypointIndx += WaypointSelector;
                                if (WaypointIndx >= waypoints.Count)
                                {
                                    switch (DriveMode)
                                    {
                                        case EDriveMode.OneWay:
                                            Busy = false;
                                            break;
                                        case EDriveMode.Patrol:
                                            WaypointSelector = -1;
                                            if (waypoints.Count > 1)
                                                WaypointIndx -= 2;
                                            else
                                                Busy = false;
                                            break;
                                        case EDriveMode.Cicrle:
                                            WaypointIndx = 0;
                                            if (waypoints.Count < 2)
                                                Busy = false;
                                            break;
                                    }
                                }
                                else if (WaypointIndx < 0)
                                {
                                    WaypointSelector = 1;
                                    WaypointIndx = 0;
                                }
                            }
                        }
                    }
                    Time = 0;
                    Manning = false;
                    RC.HandBrake = true;
                }
            }
            class CruiseControlUnit : BaseClass, IInputData
            {
                int CycleNum;
                IEnumerator<bool> StMachine;
                bool EnableState;
                readonly IInputData UserInput;
                readonly GridDynamicsWatcher Dynamics;
                readonly OrientationComputer Orientation;
                float HeadingYaw;
                float VelocitySign;

                public int SkipCycles;
                public float WS { get; set; }
                public float AD { get; set; }
                public float CSpacebar { get; set; }
                public float QE { get; set; }
                public bool Manning { get; set; }
                public bool Busy { get; set; }
                public double SpeedKPH { get; set; }
                public bool Enabled;

                public CruiseControlUnit(Program parent, IInputData userInput, GridDynamicsWatcher dynamics, OrientationComputer orientation,
                int skipCycles = 0, int skipOffset = 0) : base(parent)
                {
                    UserInput = userInput;
                    Dynamics = dynamics;
                    Orientation = orientation;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!StMachine.MoveNext())
                                StMachine = UpdateIterator();
                        }
                        else if (EnableState)
                        {
                            Busy = false;
                            Manning = false;
                            WS = AD = 0;
                            StMachine = UpdateIterator();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateIterator()
                {
                    Manning = true;
                    SpeedKPH = Dynamics.Velocity.LateralAbs * 3.6 / 0.9;
                    VelocitySign = WS = Math.Sign(Dynamics.Velocity.Backward);
                    HeadingYaw = (float)Orientation.Yaw;
                    AD = 0;
                    Busy = true;
                    while (Busy)
                    {
                        for (; CycleNum < SkipCycles; ++CycleNum)
                            yield return true;
                        CSpacebar = 0;
                        if (UserInput.WS != 0 || UserInput.CSpacebar > 0)
                        {
                            SpeedKPH = Dynamics.Velocity.LateralAbs * 3.6 / 0.9;
                            VelocitySign = Math.Sign(Dynamics.Velocity.Backward);
                            WS = UserInput.WS != 0 ? Math.Sign(UserInput.WS) + UserInput.WS : VelocitySign;
                            CSpacebar = UserInput.CSpacebar;
                        }
                        else
                            WS = VelocitySign;
                        if (UserInput.AD != 0)
                        {
                            HeadingYaw = (float)Orientation.Yaw;
                            AD = UserInput.AD;
                        }
                        else
                            AD = -VelocitySign * MathHelper.Clamp(MathHelper.WrapAngle(HeadingYaw - (float)Orientation.Yaw) * 5f, -1, 1);
                        if (Dynamics.Velocity.Scalar < 1)
                            Busy = false;
                        else
                            yield return true;
                    }
                    Manning = false;
                    RC.HandBrake = true;
                }
            }

            class GridDynamicsWatcher : BaseClass
            {
                int CycleNum;
                IEnumerator<bool> StMachine;

                public class VelocitiesCollection
                {
                    public Vector3D Vector { get; set; }
                    public double Forward { get; set; }
                    public double Right { get; set; }
                    public double Down { get; set; }
                    public double Backward { get { return -Forward; } }
                    public double Left { get { return -Right; } }
                    public double Up { get { return -Down; } }
                    public double SideAbs { get; set; }
                    public double LateralAbs { get; set; }
                    public double VerticalAbs { get; set; }
                    public double Scalar { get; set; }
                    public double NaturalPitchDegree;
                    public VelocitiesCollection() { }
                    public void Update(IMyShipController rc)
                    {
                        Vector = rc.GetShipVelocities().LinearVelocity;
                        Scalar = Vector.Length();
                        Forward = Vector.Dot(rc.WorldMatrix.Forward);
                        Right = Vector.Dot(rc.WorldMatrix.Right);
                        Down = Vector.Dot(rc.WorldMatrix.Down);
                        if (NaturalPitchDegree != 0)
                        {
                            double pScalar = new Vector2D(Forward, Down).Length();
                            Down -= Math.Sin(MathHelperD.ToRadians(NaturalPitchDegree)) * pScalar * Math.Sign(Forward);
                            Forward += Math.Abs(Math.Sin(MathHelperD.ToRadians(NaturalPitchDegree))) * pScalar;
                        }
                        LateralAbs = Math.Abs(Forward);
                        VerticalAbs = Math.Abs(Down);
                        SideAbs = Math.Abs(Right);
                    }
                    public void Reset()
                    {
                        Forward = Right = Down = SideAbs = LateralAbs = VerticalAbs = Scalar = 0;
                        Vector = Vector3D.Zero;
                    }
                }
                public class AccelerationCollection
                {
                    Vector3D VelocityVPrev;
                    double VelocitySPrev;
                    public Vector3D VectorMoment { get; set; }
                    public double AvrEMA { get; set; }
                    public double Moment { get; set; }
                    public double EMA_A;
                    public AccelerationCollection(double emaA = 0.2)
                    {
                        EMA_A = emaA;
                    }
                    public void Update(VelocitiesCollection velocity, double dT)
                    {
                        VectorMoment = (velocity.Vector - VelocityVPrev) / dT;
                        Moment = (velocity.Scalar - VelocitySPrev) / dT;
                        AvrEMA = EMA_A * Moment + (1 - EMA_A) * AvrEMA;
                        VelocityVPrev = velocity.Vector;
                        VelocitySPrev = velocity.Scalar;
                    }
                    public void Reset()
                    {
                        AvrEMA = Moment = 0;
                        VectorMoment = Vector3D.Zero;
                    }
                }

                public int SkipCycles;
                public VelocitiesCollection Velocity { get; set; }
                public AccelerationCollection Acceleration { get; set; }
                public double NaturalPitchDegree { get { return Velocity.NaturalPitchDegree; } set { Velocity.NaturalPitchDegree = value; } }
                public double dT = 0.016;

                public GridDynamicsWatcher(Program parent, int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    Velocity = new VelocitiesCollection();
                    Acceleration = new AccelerationCollection();
                    StMachine = UpdateIterator();
                }
                public void Reset()
                {
                    Velocity.Reset();
                    Acceleration.Reset();
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    double time = dT;
                    for (; CycleNum < SkipCycles; ++CycleNum)
                    {
                        time += dT;
                        yield return true;
                    }
                    CycleNum = 0;
                    Velocity.Update(RC);
                    Acceleration.Update(Velocity, time);
                }
            }

            class GridMassWatcher : BaseClass
            {
                int CycleNum;
                IEnumerator<bool> StMachine;
                bool _BaseMassChanged;
                bool _PhysicalMassChanged;
                MyShipMass MassesPrev;
                float[] ChangeIgnorArr;

                public int SkipCycles;
                public MyShipMass Masses { get; set; }
                public bool BaseMassChanged
                {
                    get
                    {
                        bool temp = _BaseMassChanged;
                        _BaseMassChanged = false;
                        return temp;
                    }
                }
                public bool PhysicalMassChanged
                {
                    get
                    {
                        bool temp = _PhysicalMassChanged;
                        _PhysicalMassChanged = false;
                        return temp;
                    }
                }

                public GridMassWatcher(Program parent, int skipCycles = 0, int skipOffset = 0) : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                }
                public void InitMasses()
                {
                    MassesPrev = RC.CalculateShipMass();
                }
                public void SetIgnorArray(float[] array)
                {
                    ChangeIgnorArr = array;
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;
                    Masses = RC.CalculateShipMass();
                    double temp;
                    temp = Math.Abs(MassesPrev.BaseMass - Masses.BaseMass);
                    if (temp == 0)
                        _BaseMassChanged = false;
                    else
                    {
                        _BaseMassChanged = true;
                        if (ChangeIgnorArr != null)
                            foreach (var d in ChangeIgnorArr)
                                if (temp == d)
                                {
                                    _BaseMassChanged = false;
                                    break;
                                }
                    }

                    temp = Math.Abs(MassesPrev.PhysicalMass - Masses.PhysicalMass);
                    if (temp == 0)
                        _PhysicalMassChanged = false;
                    else
                    {
                        _PhysicalMassChanged = true;
                        if (ChangeIgnorArr != null)
                            foreach (var d in ChangeIgnorArr)
                                if (temp == d)
                                {
                                    _PhysicalMassChanged = false;
                                    break;
                                }
                    }

                    MassesPrev = Masses;
                }
            }

            class GravityWatcher : BaseClass
            {
                int CycleNum;
                IEnumerator<bool> StMachine;
                double MagnitudePrev;
                bool _MagnitudeChanged;

                public int SkipCycles;
                public Vector3 Direction { get; set; }
                public double Magnitude { get; set; }
                public bool MagnitudeChanged
                {
                    get
                    {
                        bool temp = _MagnitudeChanged;
                        _MagnitudeChanged = false;
                        return temp;
                    }
                }
                public bool ArtificialStongerNatural { get; set; }

                public GravityWatcher(Program parent, int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;
                    ArtificialStongerNatural = RC.GetNaturalGravity().LengthSquared() < RC.GetArtificialGravity().LengthSquared();
                    Direction = RC.GetTotalGravity();
                    Magnitude = Direction.Length();
                    _MagnitudeChanged = Magnitude != MagnitudePrev;
                    MagnitudePrev = Magnitude;
                }
            }

            class OrientationComputer : BaseClass
            {
                int CycleNum;
                IEnumerator<bool> StMachine;

                public int SkipCycles;
                public double Roll { get; set; }
                public double Yaw { get; set; }
                public double Pitch { get; set; }
                public Vector3D RollYawPitch { get; set; }
                public OrientationComputer(Program parent, int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                }
                public void Update(Vector3D gravity, bool skipYaw = false)
                {
                    if (Ready)
                    {
                        if (StMachine == null)
                            StMachine = UpdateIterator(gravity, skipYaw);
                        if (!StMachine.MoveNext())
                            StMachine = null;
                    }
                }
                IEnumerator<bool> UpdateIterator(Vector3D gravity, bool skipYaw)
                {
                    Vector3D fow = RC.WorldMatrix.Forward;
                    Vector3D up = RC.WorldMatrix.Up;
                    Vector3D left = RC.WorldMatrix.Left;
                    Vector3D result = Vector3D.Zero;
                    int temp = Math.Max(SkipCycles - (skipYaw ? 1 : 2), 0);
                    int temp2 = SkipCycles - temp;
                    for (; CycleNum < temp; ++CycleNum)
                        yield return true;
                    CycleNum = 0;

                    if (!Vector3D.IsZero(gravity))
                    {
                        gravity.Normalize();
                        result.Z = -Math.Asin(MathHelperD.Clamp(fow.Dot(gravity), -1.0, 1.0));
                        if (temp2 > 1) yield return true;

                        gravity.Normalize();
                        result.X = Math.Asin(MathHelperD.Clamp(-gravity.Dot(left), -1.0, 1.0));
                        if (gravity.Dot(up) > 0)
                            result.X = Math.Sign(result.X) * (Math.PI - Math.Abs(result.X));
                        if (temp2 > 0) yield return true;

                        if (!skipYaw)
                        {
                            gravity.Normalize();
                            Vector3D gnNorth = Vector3D.Normalize(Vector3D.Reject(-Vector3D.UnitY, gravity));
                            Vector3D gnForwad = Vector3D.Normalize(Vector3D.Reject(fow, gravity));
                            result.Y = Math.Acos(MathHelperD.Clamp(gnForwad.Dot(gnNorth), -1.0, 1.0));
                            if (Math.Abs(result.X) < MathHelperD.PiOver2)
                            {
                                if (left.Dot(gnNorth) < 0)
                                    result.Y = -result.Y;
                            }
                            else if (left.Dot(gnNorth) > 0)
                                result.Y = -result.Y;
                        }
                    }
                    Roll = result.X;
                    Yaw = result.Y;
                    Pitch = result.Z;
                    RollYawPitch = new Vector3D(Roll, Yaw, Pitch);
                }
            }

            class CameraScheduler : BaseClass
            {
                class CameraWrapper
                {
                    public IMyCameraBlock Camera { get; set; }
                    public Vector3D LocalPosition { get; set; }
                    public CameraWrapper(IMyCameraBlock camera, IMyShipController anchor)
                    {
                        Camera = camera;
                        Vector3D temp1, temp2;
                        temp2 = Camera.GetPosition() - anchor.CubeGrid.WorldVolume.Center;
                        temp1.X = anchor.WorldMatrix.Right.Dot(temp2);
                        temp1.Y = anchor.WorldMatrix.Up.Dot(temp2);
                        temp1.Z = anchor.WorldMatrix.Backward.Dot(temp2);
                        LocalPosition = temp1;
                    }
                }

                readonly List<CameraWrapper> Cameras;
                readonly GridDynamicsWatcher Dynamics;
                int CycleNum;
                IEnumerator<bool> StMachine;
                double RaycastDistance = 100;
                double Time;
                int CameraIndex;
                int Odd;
                Vector3D SurfaceHitPosPrev;

                new public bool Ready { get { return (this as BaseClass).Ready && Cameras.Count > 0; } }
                public int SkipCycles;
                public double CalcSurfaceAfter;
                public double dT = 0.016;
                public Vector3D SurfaceTilt { get; set; }
                public Vector3D SurfaceBank { get; set; }
                public MyDetectedEntityType SurfaceType { get; set; }
                public double Altitude { get; set; }
                public double RaycastDistanceUsed { get; set; }
                public int DataAge { get; set; }

                public CameraScheduler(Program parent, GridDynamicsWatcher dynamics, int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    Dynamics = dynamics;
                    Cameras = new List<CameraWrapper>();
                    StMachine = UpdateIterator();
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    Cameras.Clear();
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    if (RC != null)
                    {
                        List<IMyCameraBlock> allCameras = new List<IMyCameraBlock>();
                        Parent.GridTerminalSystem.GetBlocksOfType(allCameras,
                        block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                        foreach (var camera in allCameras)
                        {
                            if (RC.Orientation.TransformDirectionInverse(camera.Orientation.Forward) == Base6Directions.Direction.Down)
                            {
                                camera.EnableRaycast = true;
                                CameraWrapper newCamera = new CameraWrapper(camera, RC);
                                Cameras.Add(newCamera);
                            }
                        }
                    }
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                    {
                        ++DataAge;
                        Time += dT;
                        yield return true;
                    }
                    CycleNum = 0;

                    ++DataAge;
                    Time += dT;
                    Odd = (Odd + 1) % 2;
                    Vector3D ScanDirection = Vector3D.Lerp(RC.WorldMatrix.Down, RC.WorldMatrix.Forward, 0.2 + Odd / 5.0);
                    // calc raycast direction as derivative from velocity and gravity vectors,but in 44.9 deg cone
                    if (Dynamics.Velocity.Scalar > 0.5)
                    {
                        double Y = Math.Acos(RC.WorldMatrix.Down.Dot(Vector3D.Normalize(Dynamics.Velocity.Vector)));
                        if (Y < 0.783652834148)
                            ScanDirection = Dynamics.Velocity.Vector;
                        else if (Y < 2.3561944902) // from 44.9 to 135 degree
                        {
                            double CC = Math.Sin(Y) / Math.Sin((Math.PI - Y) / 2.0);
                            double C1 = 0.7070944397373547 / Math.Sin(0.78538071010493 + Y / 2.0);// sin(PI/4)/sin(PI/4+Y/2)
                            ScanDirection = Vector3D.Lerp(RC.WorldMatrix.Down, Vector3D.Normalize(Dynamics.Velocity.Vector), C1 / CC);
                        }
                    }
                    SurfaceTilt = Vector3D.Zero;
                    SurfaceType = MyDetectedEntityType.None;
                    MyDetectedEntityInfo hit = TryRaycast(RaycastDistance, ScanDirection);
                    if (!hit.IsEmpty())
                    {
                        Vector3D unused = hit.HitPosition.Value - Cameras[CameraIndex].Camera.GetPosition();
                        Vector3D rayCastDistance = hit.HitPosition.Value - (RC.CubeGrid.WorldVolume.Center + RC.WorldMatrix.Down * RC.CubeGrid.WorldVolume.Radius);
                        RaycastDistance = rayCastDistance.Length() * 1.05 + 5;
                        SurfaceType = hit.Type;
                        Altitude = RC.WorldMatrix.Down.Dot(rayCastDistance);
                        DataAge = 0;

                        if (Altitude <= CalcSurfaceAfter && !Vector3D.IsZero(SurfaceHitPosPrev))
                            SurfaceTilt = hit.HitPosition.Value - SurfaceHitPosPrev;
                        SurfaceHitPosPrev = hit.HitPosition.Value;
                    }
                    else
                    {
                        RaycastDistance = Math.Min(RaycastDistance + 25, 1000.0);
                        SurfaceHitPosPrev = Vector3D.Zero;
                    }
                    yield return true;

                    ++DataAge;
                    Time += dT;
                    // raycast in side-to-side vectors and calc SurfaceBank
                    SurfaceBank = Vector3D.Zero;
                    if (Altitude <= CalcSurfaceAfter && !hit.IsEmpty())
                    {
                        Vector3D perpendicular = Vector3D.Normalize(ScanDirection.Cross(RC.WorldMatrix.Down));
                        Vector3D perpendicularDiagonal;
                        if (perpendicular.IsValid())
                            perpendicularDiagonal = Vector3D.Lerp(RC.WorldMatrix.Down, perpendicular, 0.15);
                        else
                            perpendicularDiagonal = Vector3D.Lerp(RC.WorldMatrix.Down, RC.WorldMatrix.Left, 0.15);
                        hit = TryRaycast(RaycastDistance, perpendicularDiagonal);
                        if (!hit.IsEmpty())
                        {
                            Vector3D currentHit = hit.HitPosition.Value;
                            if (perpendicular.IsValid())
                                perpendicularDiagonal = Vector3D.Lerp(RC.WorldMatrix.Down, -perpendicular, 0.15);
                            else
                                perpendicularDiagonal = Vector3D.Lerp(RC.WorldMatrix.Down, RC.WorldMatrix.Right, 0.15);
                            hit = TryRaycast(RaycastDistance, perpendicularDiagonal);
                            if (!hit.IsEmpty())
                                SurfaceBank = hit.HitPosition.Value - currentHit;
                        }
                    }

                    if (Time >= 1.0)
                    {
                        RaycastDistanceUsed = 0;
                        Time = 0;
                    }
                }
                MyDetectedEntityInfo TryRaycast(double distance, Vector3D worldDirection)
                {
                    MyDetectedEntityInfo result = new MyDetectedEntityInfo();
                    if (RaycastDistanceUsed + distance <= 2000)
                    {
                        double minAngle = double.MaxValue;
                        CameraIndex = -1;
                        Vector3D localDirection = -Vector3D.Normalize(Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(RC.WorldMatrix)));
                        for (int i = 0; i < Cameras.Count; ++i)
                        {
                            double temp = localDirection.Dot(Cameras[i].LocalPosition);
                            if (temp < minAngle && Cameras[i].Camera.IsWorking && Cameras[i].Camera.CanScan(distance))
                            {
                                minAngle = temp;
                                CameraIndex = i;
                            }
                        }
                        if (CameraIndex >= 0)
                        {
                            RaycastDistanceUsed += distance;
                            localDirection = Vector3D.Normalize(Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(Cameras[CameraIndex].Camera.WorldMatrix)));
                            result = Cameras[CameraIndex].Camera.Raycast(distance, localDirection);

                        }
                    }
                    return result;
                }
            }

            class TractionControlSystem : BaseClass
            {
                class SuspensionWrapper
                {
                    public IMyMotorSuspension Obj { get; }
                    public Base6Directions.Direction OrientationInVehicle { get; }
                    public Vector3D WheelPositionAgainstCoM { get; set; }
                    public Vector3D WheelPositionAgainstRef { get; set; }
                    public double WheelPositionAgainstVelocity { get; set; }
                    public double HeightOffsetMin { get; }
                    public double HeightOffsetMax { get; }
                    public double HeightOffsetRange { get; }
                    public double WheelRadius { get; }
                    public double PropulsionSign { get; }
                    public bool IsSubgrid { get; }
                    public double LeftMaxSteerAngle;
                    public double RightMaxSteerAngle;
                    public double TurnRadiusCurrent;
                    public double TurnRadiusLeftMin;
                    public double TurnRadiusRightMin;
                    public double WeightDistributionRatio;
                    public double BrakeFrictionDistributionRatio;
                    public double SpeedLimit { get { return Obj.GetValueFloat("Speed Limit"); } set { Obj.SetValueFloat("Speed Limit", (float)value); } }
                    public double PropulsionOverride { get { return Obj.GetValueFloat("Propulsion override"); } set { Obj.SetValueFloat("Propulsion override", (float)value); } }
                    public double SteerOverrie { get { return Obj.GetValueFloat("Steer override"); } set { Obj.SetValueFloat("Steer override", (float)value); } }
                    public double Power { get { return Obj.Power; } set { Obj.Power = (float)value; } }
                    public double Friction { get { return Obj.Friction; } set { Obj.Friction = (float)value; } }
                    public double Strength { get { return Obj.Strength; } set { Obj.Strength = (float)value; } }
                    public double Height { get { return Obj.Height; } set { Obj.Height = (float)value; } }
                    public double MaxSteerAngle { get { return Obj.MaxSteerAngle; } set { Obj.MaxSteerAngle = (float)value; } }
                    public SuspensionWrapper(IMyMotorSuspension suspension, Base6Directions.Direction orientation, bool subgrid = false)
                    {
                        Obj = suspension;
                        OrientationInVehicle = orientation;
                        IsSubgrid = subgrid;
                        if (orientation == Base6Directions.Direction.Left)
                            PropulsionSign = -1;
                        else if (orientation == Base6Directions.Direction.Right)
                            PropulsionSign = 1;
                        HeightOffsetMin = suspension.GetMinimum<float>("Height");
                        HeightOffsetMax = suspension.GetMaximum<float>("Height");
                        HeightOffsetRange = HeightOffsetMax - HeightOffsetMin;
                        if (suspension.CubeGrid.GridSizeEnum == MyCubeSize.Small)
                        {
                            if (suspension.BlockDefinition.SubtypeName.Contains("5x5")) WheelRadius = 1.25;
                            else if (suspension.BlockDefinition.SubtypeName.Contains("3x3")) WheelRadius = 0.75;
                            else if (suspension.BlockDefinition.SubtypeName.Contains("2x2")) WheelRadius = 0.5;// modded
                            else if (suspension.BlockDefinition.SubtypeName.Contains("1x1")) WheelRadius = 0.25;
                            else // some other modded wheels
                                WheelRadius = suspension.IsAttached ? suspension.Top.WorldVolume.Radius * 0.79 / MathHelper.Sqrt2 : 0;
                        }
                        else
                        {
                            if (suspension.BlockDefinition.SubtypeName.Contains("5x5")) WheelRadius = 6.25;
                            else if (suspension.BlockDefinition.SubtypeName.Contains("3x3")) WheelRadius = 3.75;
                            else if (suspension.BlockDefinition.SubtypeName.Contains("2x2")) WheelRadius = 2.5;// modded
                            else if (suspension.BlockDefinition.SubtypeName.Contains("1x1")) WheelRadius = 1.25;
                            else // some other modded wheels
                                WheelRadius = suspension.IsAttached ? suspension.Top.WorldVolume.Radius * 0.79 / MathHelper.Sqrt2 : 0;
                        }
                    }
                    public Vector3 GetVelocityAtPoint(IMyShipController anchor)
                    {
                        Vector3 value = Vector3D.Zero;
                        if (Obj.IsAttached)
                        {
                            Vector3 v = Obj.Top.GetPosition() - anchor.CenterOfMass;
                            value = anchor.GetShipVelocities().LinearVelocity + anchor.GetShipVelocities().AngularVelocity.Cross(v);
                        }
                        return value;
                    }
                    public bool AddTopPart()
                    {
                        Obj.ApplyAction("Add Top Part");
                        return Obj.IsAttached;
                    }
                    public void UpdateLocalPosition(IMyShipController anchor, Vector3D focalPointRef)
                    {
                        if (Obj.IsAttached)
                        {
                            Vector3D temp1, temp2;
                            temp2 = Obj.Top.GetPosition() - anchor.CenterOfMass;
                            temp1.X = anchor.WorldMatrix.Right.Dot(temp2);
                            temp1.Y = anchor.WorldMatrix.Up.Dot(temp2);
                            temp1.Z = anchor.WorldMatrix.Backward.Dot(temp2);
                            WheelPositionAgainstCoM = temp1;
                            temp2 = Obj.Top.GetPosition() - focalPointRef;
                            temp1.X = anchor.WorldMatrix.Right.Dot(temp2);
                            temp1.Y = anchor.WorldMatrix.Up.Dot(temp2);
                            temp1.Z = anchor.WorldMatrix.Backward.Dot(temp2);
                            WheelPositionAgainstRef = temp1;
                        }
                        else
                            WheelPositionAgainstRef = WheelPositionAgainstCoM = Vector3D.Zero;
                    }
                    public void UpdatePositionVelocity(Vector3D velocity)
                    {
                        if (Obj.IsAttached)
                            WheelPositionAgainstVelocity = velocity.Dot(WheelPositionAgainstCoM);
                        else
                            WheelPositionAgainstVelocity = 0;
                    }
                }

                readonly List<SuspensionWrapper> WheelSuspensions;
                readonly List<IMyShipController> SlaveShipControllers;
                readonly GridDynamicsWatcher Dynamics;
                readonly GravityWatcher Gravity;
                readonly OrientationComputer Orientation;
                readonly CameraScheduler Camera;
                IInputData UserInput;
                int FCycleNum;
                int TCycleNum;
                int HCycleNum;
                int StrCycleNum;
                int SpdCycleNum;
                int SteCycleNum;
                int BrCycleNum;
                int WlAdrCycleNum;
                IEnumerator<bool> FStMachine;
                IEnumerator<bool> TStMachine;
                IEnumerator<bool> HStMachine;
                IEnumerator<bool> StrStMachine;
                IEnumerator<bool> SpdStMachine;
                IEnumerator<bool> SteStMachine;
                IEnumerator<bool> BrStMachine;
                IEnumerator<bool> WlAdrStMachine;
                double CurrentTorque;
                double PropulsionAmp = 1;
                double GridSprungWeight;
                double StrenghtDelta = 2;
                double HeightOffsetDelta = 0.1;
                double _Strenght;
                double _HeightOffset;
                bool BusyWithHeight;
                bool BusyWithStrenght;
                bool EnableState;
                readonly string SubgridWheelsGroupName;

                public bool Enabled;
                new public bool Ready
                {
                    get
                    {
                        return (this as BaseClass).Ready
                        && UserInput.Ready && Dynamics.Ready && Gravity.Ready && Orientation.Ready
                        && WheelSuspensions.Count > 0;
                    }
                }
                public bool Busy { get { return BusyWithHeight || BusyWithStrenght; } }
                public bool UseAdaptiveSteering;
                public bool UseAckermannSteering;
                public bool UseDASAirShock;
                public bool UseHillDescentControl;
                public bool ForceFullOverride;
                public int FrictionSkipCycles;
                public int TorqueSkipCycles;// 'power' property.
                public int HeightOfstSkipCycles;
                public int StrenghtSkipCycles;
                public int SpeedLimitSkipCycles;
                public int SteerSkipCycles;
                public int BrakesSkipCycles;
                public int WheelAddSkipCycles;
                public double PowerLimit;
                public double ForwardSpeedLimitKPH;
                public double BackwardSpeedLimitKPH;
                public double MaxForwardAcceleration;
                public double MaxBackwardAcceleration;
                public double MaxSteerAngleDegree;
                public double Strenght
                {
                    get { return _Strenght; }
                    set
                    {
                        StrenghtDelta = Math.Abs(value - _Strenght);
                        _Strenght = value;
                    }
                }
                public double HeightOffset
                {
                    get { return _HeightOffset; }
                    set
                    {
                        if (value >= 0)
                            HeightOffsetDelta = Math.Abs(value - _HeightOffset);
                        _HeightOffset = value;
                    }
                }
                public double ValueChangeRate;
                public double VehicleTurnRadiusRight { get; private set; }
                public double VehicleTurnRadiusLeft { get; private set; }
                public double CoMToWheelsBaseAngle { get; private set; }
                public AckermannFocalPointRef AckFocal;
                public TractionControlSystem(Program parent, GridDynamicsWatcher dynamics, GravityWatcher gravity,
                OrientationComputer orientation, CameraScheduler camera, string sbgrdWhlGrpNm = "",
                int fCycles = 0, int fOffset = 0, int tCycles = 0, int tOffset = 0, int hCycles = 0, int hOffset = 0,
                int strCycles = 0, int strOffset = 0, int sCycles = 0, int sOffset = 0, int steCycles = 0, int steOffset = 0,
                int bCycles = 0, int bOffset = 0, int wCycles = 0, int wOffset = 0) : base(parent)
                {
                    Dynamics = dynamics;
                    Gravity = gravity;
                    Orientation = orientation;
                    Camera = camera;
                    SubgridWheelsGroupName = sbgrdWhlGrpNm;
                    FrictionSkipCycles = fCycles;
                    FCycleNum = fOffset;
                    TorqueSkipCycles = tCycles;
                    TCycleNum = tOffset;
                    HeightOfstSkipCycles = hCycles;
                    HCycleNum = hOffset;
                    StrenghtSkipCycles = strCycles;
                    StrCycleNum = strOffset;
                    SpeedLimitSkipCycles = sCycles;
                    SpdCycleNum = sOffset;
                    SteerSkipCycles = steCycles;
                    SteCycleNum = steOffset;
                    BrakesSkipCycles = bCycles;
                    BrCycleNum = bOffset;
                    WheelAddSkipCycles = wCycles;
                    WlAdrCycleNum = wOffset;
                    WheelSuspensions = new List<SuspensionWrapper>();
                    SlaveShipControllers = new List<IMyShipController>();
                    InitStateMachines();
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    WheelSuspensions.Clear();
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    if (RC != null)
                    {
                        List<IMyMotorSuspension> allWheels = new List<IMyMotorSuspension>();
                        Parent.GridTerminalSystem.GetBlocksOfType(allWheels,
                        block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                        foreach (var suspension in allWheels)
                        {
                            Base6Directions.Direction blockOrientation = RC.Orientation.TransformDirectionInverse(suspension.Orientation.Up);
                            if (blockOrientation == Base6Directions.Direction.Left || blockOrientation == Base6Directions.Direction.Right)
                            {
                                suspension.AirShockEnabled = !UseDASAirShock;
                                SuspensionWrapper newSuspensionData = new SuspensionWrapper(suspension, blockOrientation)
                                {
                                    PropulsionOverride = 0,
                                    SteerOverrie = 0
                                };
                                WheelSuspensions.Add(newSuspensionData);
                            }
                        }

                        IMyBlockGroup subgridWheelsGroup = Parent.GridTerminalSystem.GetBlockGroupWithName(SubgridWheelsGroupName);
                        if (subgridWheelsGroup != null)
                        {
                            SuspensionWrapper newSuspension;
                            List<IMyCubeGrid> wheeledSubgrids = new List<IMyCubeGrid>();
                            List<IMyShipController> slaveControllers = new List<IMyShipController>();
                            allWheels.Clear();
                            subgridWheelsGroup.GetBlocksOfType(allWheels);
                            foreach (var suspension in allWheels)
                            {
                                if (suspension.CubeGrid != Parent.Me.CubeGrid)
                                {
                                    if (suspension.WorldMatrix.Up.Dot(RC.WorldMatrix.Left) > 0.7)
                                    {
                                        suspension.AirShockEnabled = !UseDASAirShock;
                                        newSuspension = new SuspensionWrapper(suspension, Base6Directions.Direction.Left, true);
                                        WheelSuspensions.Add(newSuspension);
                                        if (wheeledSubgrids.IndexOf(suspension.CubeGrid) == -1)
                                            wheeledSubgrids.Add(suspension.CubeGrid);
                                    }
                                    else if (suspension.WorldMatrix.Up.Dot(RC.WorldMatrix.Right) > 0.7)
                                    {
                                        suspension.AirShockEnabled = !UseDASAirShock;
                                        newSuspension = new SuspensionWrapper(suspension, Base6Directions.Direction.Right, true);
                                        WheelSuspensions.Add(newSuspension);
                                        if (wheeledSubgrids.IndexOf(suspension.CubeGrid) == -1)
                                            wheeledSubgrids.Add(suspension.CubeGrid);
                                    }
                                }
                            }
                            foreach (var grid in wheeledSubgrids)
                            {
                                slaveControllers.Clear();
                                Parent.GridTerminalSystem.GetBlocksOfType(slaveControllers, block => block.CubeGrid == grid && !Utils.IsIgnore(block));
                                foreach (var controller in slaveControllers)
                                    if (SlaveShipControllers.IndexOf(controller) == -1)
                                        SlaveShipControllers.Add(controller);
                            }
                        }
                        UpdateSuspensionCofiguration();
                    }
                }
                public void UpdateSprungWeight(double gridPhysMass)
                {
                    double unsprungMass = 0;
                    foreach (var suspension in WheelSuspensions)
                        if (suspension.Obj.IsAttached)
                            unsprungMass += suspension.Obj.Top.Mass;
                    GridSprungWeight = (gridPhysMass - unsprungMass) * Gravity.Magnitude;
                }
                public void UpdateSuspensionCofiguration()
                {
                    double maxL = 0;
                    double maxR = 0;
                    double wheelFocus;
                    double rearHalfZ = double.MinValue;
                    double frontHalfZ = double.MaxValue;
                    double wheelsBaseZ;
                    double weightsSum = 0;
                    double comHeight = 0;

                    VehicleTurnRadiusLeft = VehicleTurnRadiusRight = 0;
                    foreach (var suspension in WheelSuspensions)
                        if (suspension.Obj.IsAttached)
                        {
                            switch (AckFocal)
                            {
                                case AckermannFocalPointRef.CoM:
                                    suspension.UpdateLocalPosition(RC, RC.CenterOfMass);
                                    break;
                                case AckermannFocalPointRef.RC:
                                    suspension.UpdateLocalPosition(RC, RC.GetPosition());
                                    break;
                                case AckermannFocalPointRef.BSphere:
                                    suspension.UpdateLocalPosition(RC, RC.CubeGrid.WorldVolume.Center);
                                    break;
                            }
                            if (Math.Sign(suspension.WheelPositionAgainstRef.Z) != Math.Sign(suspension.WheelPositionAgainstCoM.Z))
                                suspension.Obj.InvertSteer = true;
                            else
                                suspension.Obj.InvertSteer = false;

                            comHeight = Math.Max(comHeight, -suspension.WheelPositionAgainstCoM.Y + suspension.WheelRadius
                            - Math.Max(_HeightOffset, suspension.HeightOffsetMin));
                            if (suspension.OrientationInVehicle == Base6Directions.Direction.Left)
                                maxL = Math.Max(maxL, -suspension.WheelPositionAgainstCoM.X);
                            else if (suspension.OrientationInVehicle == Base6Directions.Direction.Right)
                                maxR = Math.Max(maxR, suspension.WheelPositionAgainstCoM.X);

                            if (suspension.Obj.Steering)
                            {
                                wheelFocus = Math.Abs(suspension.WheelPositionAgainstRef.Z / Math.Tan(MathHelperD.ToRadians(MaxSteerAngleDegree)));
                                if (VehicleTurnRadiusLeft > suspension.WheelPositionAgainstRef.X - wheelFocus)
                                    VehicleTurnRadiusLeft = suspension.WheelPositionAgainstRef.X - wheelFocus;
                                if (VehicleTurnRadiusRight < suspension.WheelPositionAgainstRef.X + wheelFocus)
                                    VehicleTurnRadiusRight = suspension.WheelPositionAgainstRef.X + wheelFocus;
                            }
                            rearHalfZ = Math.Max(rearHalfZ, suspension.WheelPositionAgainstCoM.Z);
                            frontHalfZ = Math.Min(frontHalfZ, suspension.WheelPositionAgainstCoM.Z);
                        }
                    CoMToWheelsBaseAngle = Math.Atan(comHeight / Math.Min(maxL, maxR));
                    wheelsBaseZ = rearHalfZ - frontHalfZ;
                    double weightRearRatio = Math.Abs(frontHalfZ / wheelsBaseZ);
                    double weightFrontRatio = Math.Abs(rearHalfZ / wheelsBaseZ);
                    double frictionLowerLever = 0.5 * rearHalfZ / -frontHalfZ;
                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.Steering)
                        {
                            double temp1 = (VehicleTurnRadiusLeft - suspension.WheelPositionAgainstRef.X);
                            double temp2 = (VehicleTurnRadiusRight - suspension.WheelPositionAgainstRef.X);
                            suspension.LeftMaxSteerAngle = Math.Abs(Math.Atan(suspension.WheelPositionAgainstRef.Z / temp1));
                            suspension.RightMaxSteerAngle = Math.Abs(Math.Atan(suspension.WheelPositionAgainstRef.Z / temp2));
                            suspension.TurnRadiusLeftMin = Math.Sqrt(suspension.WheelPositionAgainstRef.Z * suspension.WheelPositionAgainstRef.Z + temp1 * temp1);
                            suspension.TurnRadiusRightMin = Math.Sqrt(suspension.WheelPositionAgainstRef.Z * suspension.WheelPositionAgainstRef.Z + temp2 * temp2);
                        }
                        else
                        {
                            suspension.LeftMaxSteerAngle = suspension.RightMaxSteerAngle = 0;
                            suspension.TurnRadiusCurrent = suspension.TurnRadiusLeftMin = suspension.TurnRadiusRightMin = 1;
                        }
                        if (wheelsBaseZ > 0.1)
                        {
                            if (suspension.WheelPositionAgainstCoM.Z >= 0)
                                suspension.WeightDistributionRatio = Math.Abs((suspension.WheelPositionAgainstCoM.Z - frontHalfZ) / wheelsBaseZ * weightRearRatio);
                            else
                                suspension.WeightDistributionRatio = Math.Abs((rearHalfZ - suspension.WheelPositionAgainstCoM.Z) / wheelsBaseZ * weightFrontRatio);

                            suspension.BrakeFrictionDistributionRatio = frictionLowerLever + (1 - frictionLowerLever) * (suspension.WheelPositionAgainstCoM.Z - frontHalfZ) / wheelsBaseZ;
                        }
                        else
                        {
                            suspension.WeightDistributionRatio = 1;
                            suspension.BrakeFrictionDistributionRatio = 1;
                        }
                        weightsSum += suspension.WeightDistributionRatio;
                    }
                    foreach (var suspension in WheelSuspensions)
                    {
                        suspension.WeightDistributionRatio /= weightsSum;
                    }
                }
                public void ReleaseControl()
                {
                    foreach (var suspension in WheelSuspensions)
                    {
                        suspension.Power = 100 * PowerLimit;
                        suspension.Friction = 100;
                        suspension.SpeedLimit = ForwardSpeedLimitKPH;
                        if (UseAckermannSteering)
                            suspension.MaxSteerAngle = MathHelperD.ToRadians(MaxSteerAngleDegree);
                        suspension.PropulsionOverride = 0;
                        suspension.SteerOverrie = 0;
                        suspension.Obj.AirShockEnabled = true;
                    }
                }
                public void SetInputSource(IInputData source)
                {
                    UserInput = source;
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!FStMachine.MoveNext()) FStMachine = UpdateFriction();
                            if (!TStMachine.MoveNext()) TStMachine = UpdateTorque();
                            if (!HStMachine.MoveNext()) HStMachine = UpdateHeight();
                            if (!StrStMachine.MoveNext()) StrStMachine = UpdateStrenght();
                            if (!SpdStMachine.MoveNext()) SpdStMachine = UpdateSpeedLim();
                            if (!SteStMachine.MoveNext()) SteStMachine = UpdateSteerAngl();
                            if (!BrStMachine.MoveNext()) BrStMachine = UpdateBrakes();
                            if (!WlAdrStMachine.MoveNext()) WlAdrStMachine = UpdateWheelAdder();
                        }
                        else if (EnableState)
                        {
                            ReleaseControl();
                            InitStateMachines();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateFriction()
                {
                    int temp = FrictionSkipCycles - 1;
                    for (; FCycleNum < temp; ++FCycleNum)
                        yield return true;
                    FCycleNum = 0;
                    // calc. friction modifiers to get proper total friction point against velocity and CoM
                    double baseFriction;
                    double driftAngleSinThreshold = 0.017;
                    double wheelsBaseZ = 1, frictionLowerLever = 1, driftAngleSin = 0;
                    double rearHalfZ = double.MinValue;
                    double frontHalfZ = double.MaxValue;
                    double minBrakeFriction = double.MaxValue;
                    if (Dynamics.Velocity.Scalar > 5)
                    {
                        Vector3D velocityLocal = Vector3D.TransformNormal(Dynamics.Velocity.Vector, MatrixD.Transpose(RC.WorldMatrix));
                        velocityLocal.Y = 0;
                        velocityLocal.Normalize();
                        driftAngleSin = Math.Abs(velocityLocal.Dot(Vector3D.Right));
                        foreach (var suspension in WheelSuspensions)
                        {
                            if (suspension.Obj.IsAttached)
                            {
                                suspension.UpdatePositionVelocity(-velocityLocal);
                                rearHalfZ = Math.Max(rearHalfZ, suspension.WheelPositionAgainstVelocity);
                                frontHalfZ = Math.Min(frontHalfZ, suspension.WheelPositionAgainstVelocity);
                                minBrakeFriction = Math.Min(minBrakeFriction, suspension.BrakeFrictionDistributionRatio);
                            }
                        }
                        wheelsBaseZ = rearHalfZ - frontHalfZ;
                        frictionLowerLever = (0.95 - MathHelperD.Clamp(driftAngleSin * 2, 0, 0.9)) * rearHalfZ / -frontHalfZ;
                    }
                    if (wheelsBaseZ == 0)
                        wheelsBaseZ = 1;
                    baseFriction = 100 - 35 * Math.Abs(UserInput.AD);
                    if (driftAngleSin > driftAngleSinThreshold)
                        baseFriction *= (1 - MathHelperD.Clamp(driftAngleSin * 2, 0, 0.5));
                    if (FrictionSkipCycles > 0) yield return true;

                    double friction;
                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.IsAttached)
                        {
                            friction = baseFriction;
                            if (Dynamics.Velocity.Scalar > 0.5)
                            {
                                if (RC.HandBrake || UserInput.CSpacebar > 0) // friction rebalance at braking
                                    friction *= suspension.BrakeFrictionDistributionRatio;
                                else if (driftAngleSin > driftAngleSinThreshold && Dynamics.Velocity.SideAbs > 5) // friction rebalance against velocity
                                    friction *= frictionLowerLever + (1 - frictionLowerLever) * (suspension.WheelPositionAgainstVelocity - frontHalfZ) / wheelsBaseZ;
                                if (UserInput.AD != 0 && UserInput.WS < 0 && UserInput.CSpacebar > 0 && suspension.WheelPositionAgainstCoM.Z > 0 && Dynamics.Velocity.Forward > 8)
                                    friction *= minBrakeFriction * 1.4;
                                friction = Math.Max(friction, 5);
                            }
                            suspension.Friction = friction;
                        }
                    }
                }
                IEnumerator<bool> UpdateTorque()
                {
                    int temp = TorqueSkipCycles - 1;
                    for (; TCycleNum < temp; ++TCycleNum)
                    {
                        RampUpDownTorque();
                        AmplifyPropulsion();
                        yield return true;
                    }
                    TCycleNum = 0;

                    double maxTurnRadius = 1;
                    foreach (var suspension in WheelSuspensions)
                        if (suspension.Obj.IsAttached)
                            maxTurnRadius = Math.Max(maxTurnRadius, suspension.TurnRadiusCurrent);
                    if (TorqueSkipCycles > 0)
                    {
                        yield return true;
                        RampUpDownTorque();
                        AmplifyPropulsion();
                    }

                    RampUpDownTorque();
                    AmplifyPropulsion();
                    double torque;
                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.IsAttached)
                        {
                            torque = CurrentTorque * PowerLimit;
                            if (Dynamics.Velocity.Scalar > 0.5)
                                torque *= suspension.TurnRadiusCurrent / maxTurnRadius;
                            torque = Math.Min(torque, 100 * PowerLimit);
                            suspension.Power = torque;

                            if (Dynamics.Velocity.VerticalAbs < 1) // !!workaround: another Keen's bug
                            {
                                double speedLimitKPH = Dynamics.Velocity.Forward > 0 ? ForwardSpeedLimitKPH : BackwardSpeedLimitKPH;
                                double propulsion = 0;
                                if (UseHillDescentControl && Dynamics.Velocity.LateralAbs * 3.6 > speedLimitKPH && !(UserInput.WS != 0 && UserInput.CSpacebar < 0))
                                    propulsion = suspension.PropulsionSign * Math.Sign(Dynamics.Velocity.Forward);
                                else if (suspension.IsSubgrid || ForceFullOverride)
                                    propulsion = torque / 100 * suspension.PropulsionSign * UserInput.WS * PropulsionAmp;
                                propulsion = Math.Min(propulsion, PowerLimit);
                                suspension.PropulsionOverride = propulsion;
                            }
                            else
                                suspension.PropulsionOverride = 0;
                        }
                    }
                }
                IEnumerator<bool> UpdateHeight()
                {
                    for (; HCycleNum < HeightOfstSkipCycles; ++HCycleNum)
                        yield return true;
                    HCycleNum = 0;

                    double currentHeightOffset;
                    double clampedTargetHeightOffset;
                    double temp;
                    double delta = HeightOffsetDelta / ValueChangeRate * (HeightOfstSkipCycles + 1);
                    BusyWithHeight = false;
                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.IsAttached)
                        {
                            currentHeightOffset = suspension.Height;
                            clampedTargetHeightOffset = MathHelperD.Clamp(_HeightOffset, suspension.HeightOffsetMin, suspension.HeightOffsetMax);
                            temp = Math.Round(clampedTargetHeightOffset - currentHeightOffset, 2);
                            if (temp < 0)
                            {
                                currentHeightOffset = Math.Max(currentHeightOffset - delta, clampedTargetHeightOffset);
                                suspension.Height = currentHeightOffset;
                                BusyWithHeight = true;
                            }
                            else if (temp > 0)
                            {
                                currentHeightOffset = Math.Min(currentHeightOffset + delta, clampedTargetHeightOffset);
                                suspension.Height = currentHeightOffset;
                                BusyWithHeight = true;
                            }
                        }
                    }
                }
                IEnumerator<bool> UpdateStrenght()
                {
                    for (; StrCycleNum < StrenghtSkipCycles; ++StrCycleNum)
                        yield return true;
                    StrCycleNum = 0;

                    double currentStrenght;
                    double strenght;
                    double temp;
                    double delta = StrenghtDelta / ValueChangeRate * (HeightOfstSkipCycles + 1);
                    BusyWithStrenght = false;
                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.IsAttached)
                        {
                            if (_Strenght >= 0)
                                strenght = _Strenght;
                            else if (GridSprungWeight > 0)
                            {
                                strenght = Math.Sqrt(suspension.WeightDistributionRatio * GridSprungWeight);
                                if (suspension.Obj.CubeGrid.GridSizeEnum == MyCubeSize.Small)
                                {
                                    if (suspension.Obj.BlockDefinition.SubtypeName.Contains("5x5"))
                                        strenght /= 18.5;
                                    else
                                        strenght /= 15;
                                }
                                else
                                {
                                    if (suspension.Obj.BlockDefinition.SubtypeName.Contains("5x5"))
                                        strenght /= 55;
                                    else
                                        strenght /= 52.5;
                                }
                            }
                            else
                                strenght = 5;

                            currentStrenght = suspension.Strength;
                            double fallVelocityExceeding = RC.WorldMatrix.Down.Dot(suspension.GetVelocityAtPoint(RC)) / 10;
                            if (fallVelocityExceeding > 1 && UseDASAirShock)
                                currentStrenght = strenght * fallVelocityExceeding * fallVelocityExceeding;
                            else if (Dynamics.Velocity.Up > 1 && UseDASAirShock)
                                currentStrenght = strenght;
                            else
                            {
                                temp = Math.Round(strenght - currentStrenght, 2);
                                if (temp < 0)
                                {
                                    currentStrenght = Math.Max(currentStrenght - delta, strenght);
                                    BusyWithStrenght = true;
                                }
                                else if (temp > 0)
                                {
                                    currentStrenght = Math.Min(currentStrenght + delta, strenght);
                                    BusyWithStrenght = true;
                                }
                            }
                            if (Math.Abs(UserInput.AD) < 0.2) // !!workaround: steering bug
                                suspension.Strength = currentStrenght;
                        }
                    }
                }
                IEnumerator<bool> UpdateSpeedLim()
                {
                    for (; SpdCycleNum < SpeedLimitSkipCycles; ++SpdCycleNum)
                        yield return true;
                    SpdCycleNum = 0;

                    if (UserInput.WS != 0)
                        foreach (var suspension in WheelSuspensions)
                        {
                            if (suspension.Obj.IsAttached)
                            {
                                if (UserInput.CSpacebar < 0)
                                    suspension.SpeedLimit = 360 * Math.Abs(UserInput.WS);
                                else if (Dynamics.Velocity.Forward > 0)
                                    suspension.SpeedLimit = ForwardSpeedLimitKPH * Math.Abs(UserInput.WS);
                                else
                                    suspension.SpeedLimit = BackwardSpeedLimitKPH * Math.Abs(UserInput.WS);
                            }
                        }
                }
                IEnumerator<bool> UpdateSteerAngl()
                {
                    for (; SteCycleNum < SteerSkipCycles; ++SteCycleNum)
                        yield return true;
                    SteCycleNum = 0;

                    double safeTurnRadiusRight = 0;
                    double safeTurnRadiusLeft = 0;
                    bool adaptiveSteering;
                    if ((UserInput.AD != 0 && UserInput.WS != 0 && UserInput.CSpacebar > 0) || Dynamics.Velocity.LateralAbs < 0.5)
                        adaptiveSteering = false;
                    else
                        adaptiveSteering = UseAdaptiveSteering;
                    if (adaptiveSteering && Gravity.Magnitude > 0)
                    {
                        double pow = Camera.SurfaceType == MyDetectedEntityType.LargeGrid ? 2 : 1.5;
                        if (UserInput.AD < 0)
                            safeTurnRadiusLeft = Math.Pow(Dynamics.Velocity.LateralAbs, pow) / (Gravity.Magnitude * Math.Cos(MathHelperD.Clamp(CoMToWheelsBaseAngle + Orientation.Roll, 1e-6, MathHelperD.Pi / 2.1)));
                        else if (UserInput.AD > 0)
                            safeTurnRadiusRight = Math.Pow(Dynamics.Velocity.LateralAbs, pow) / (Gravity.Magnitude * Math.Cos(MathHelperD.Clamp(CoMToWheelsBaseAngle - Orientation.Roll, 1e-6, MathHelperD.Pi / 2.1)));
                    }
                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.Steering && suspension.Obj.IsAttached)
                        {
                            if (UserInput.AD != 0 && UseAckermannSteering)
                            {
                                double steerAngle;
                                if (adaptiveSteering)
                                {
                                    double turnCath;
                                    if (UserInput.AD > 0)
                                        turnCath = Math.Max(VehicleTurnRadiusRight, safeTurnRadiusRight) - suspension.WheelPositionAgainstRef.X;
                                    else
                                        turnCath = Math.Min(VehicleTurnRadiusLeft, -safeTurnRadiusLeft) - suspension.WheelPositionAgainstRef.X;
                                    steerAngle = Math.Abs(Math.Atan(suspension.WheelPositionAgainstRef.Z / turnCath));
                                    suspension.TurnRadiusCurrent = Math.Sqrt(suspension.WheelPositionAgainstRef.Z * suspension.WheelPositionAgainstRef.Z + turnCath * turnCath);
                                }
                                else
                                {
                                    if (UserInput.AD > 0)
                                    {
                                        steerAngle = suspension.RightMaxSteerAngle;
                                        suspension.TurnRadiusCurrent = suspension.TurnRadiusRightMin;
                                    }
                                    else
                                    {
                                        steerAngle = suspension.LeftMaxSteerAngle;
                                        suspension.TurnRadiusCurrent = suspension.TurnRadiusLeftMin;
                                    }
                                }
                                suspension.MaxSteerAngle = steerAngle;
                            }
                            else
                                suspension.TurnRadiusCurrent = 1;
                            if (suspension.IsSubgrid || ForceFullOverride)
                                suspension.SteerOverrie = -Math.Sign(suspension.WheelPositionAgainstRef.Z) * UserInput.AD;
                        }
                    }
                }
                IEnumerator<bool> UpdateBrakes()
                {
                    int temp = BrakesSkipCycles - 1;
                    for (; BrCycleNum < temp; ++BrCycleNum)
                        yield return true;
                    BrCycleNum = 0;

                    foreach (var suspension in WheelSuspensions)
                    {
                        if (suspension.Obj.IsAttached)
                        {
                            if (suspension.WheelPositionAgainstCoM.Z > 0 && UserInput.WS != 0 && !RC.HandBrake)
                                suspension.Obj.Brake = false;
                            else
                                suspension.Obj.Brake = true;
                        }
                    }
                    if (BrakesSkipCycles > 0) yield return true;
                    // If there slave controllers on wheeled subgrid-handle handbrakes
                    if (SlaveShipControllers.Count > 0)
                    {
                        if (UserInput.CSpacebar > 0 || RC.HandBrake)
                            foreach (var sc in SlaveShipControllers)
                                sc.HandBrake = true;
                        else
                            foreach (var sc in SlaveShipControllers)
                                sc.HandBrake = false;
                    }
                }
                IEnumerator<bool> UpdateWheelAdder()
                {
                    for (; WlAdrCycleNum < WheelAddSkipCycles; ++WlAdrCycleNum)
                        yield return true;
                    WlAdrCycleNum = 0;

                    foreach (var suspension in WheelSuspensions)
                    {
                        if (!suspension.Obj.IsAttached)
                        {
                            for (int i = 0; i <= 10; ++i)
                            {
                                if (suspension.AddTopPart())
                                    break;
                                yield return true;
                                suspension.Height = suspension.HeightOffsetMin + suspension.HeightOffsetRange / 10.0 * i;
                            }
                        }
                    }
                }
                void RampUpDownTorque()
                {
                    if ((Dynamics.Acceleration.AvrEMA < MaxForwardAcceleration && UserInput.WS < 0)
                    || (Dynamics.Acceleration.AvrEMA < MaxBackwardAcceleration && UserInput.WS > 0))
                        CurrentTorque = Math.Min(CurrentTorque * 1.05, 100);
                    else
                        CurrentTorque = Math.Max(CurrentTorque * 0.9, 5);
                }
                void AmplifyPropulsion()
                {
                    if (Dynamics.Velocity.LateralAbs * 3.6 < Math.Abs(UserInput.WS) * ForwardSpeedLimitKPH * 0.9 && UserInput.WS < 0)
                        PropulsionAmp = Math.Min(PropulsionAmp * 1.01, 360.0 / ForwardSpeedLimitKPH);
                    else if (Dynamics.Velocity.LateralAbs * 3.6 < Math.Abs(UserInput.WS) * BackwardSpeedLimitKPH * 0.9 && UserInput.WS > 0)
                        PropulsionAmp = Math.Min(PropulsionAmp * 1.01, 360.0 / BackwardSpeedLimitKPH);
                    else
                        PropulsionAmp = Math.Max(PropulsionAmp * 0.95, 1);
                }
                void InitStateMachines()
                {
                    FStMachine = UpdateFriction();
                    TStMachine = UpdateTorque();
                    HStMachine = UpdateHeight();
                    StrStMachine = UpdateStrenght();
                    SpdStMachine = UpdateSpeedLim();
                    SteStMachine = UpdateSteerAngl();
                    BrStMachine = UpdateBrakes();
                    WlAdrStMachine = UpdateWheelAdder();
                }
            }

            class FallDampeningSystem : BaseClass
            {
                int CycleNum;
                int AltitudeAge;
                IEnumerator<bool> StMachine;
                bool EnableState;
                readonly List<IMyThrust> Thrusters;
                readonly GridDynamicsWatcher Dynamics;
                readonly GravityWatcher Gravity;
                readonly GridMassWatcher Mass;
                readonly CameraScheduler Camera;
                readonly IInputData UserInput;
                double Altitude;
                double AltitudePrev;
                double EmpiricDescentVeloc;
                double Time;

                public bool Enabled;
                new public bool Ready { get { return (this as BaseClass).Ready && Thrusters.Count > 0; } }
                public bool EnoughThrust { get; set; }
                public bool UseJumpJets;
                public int SkipCycles;
                public double SafeFallVelocity;
                public double dT = 0.016;
                public FallDampeningSystem(Program parent, GridDynamicsWatcher dynamics, GravityWatcher gravity,
                GridMassWatcher masses, CameraScheduler camera, IInputData userInput = null,
                int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    Dynamics = dynamics;
                    Gravity = gravity;
                    Mass = masses;
                    Camera = camera;
                    UserInput = userInput;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    Thrusters = new List<IMyThrust>();
                    StMachine = UpdateIterator();
                    EnoughThrust = true;
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    Thrusters.Clear();
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    if (RC != null)
                    {
                        Base6Directions.Direction blockOrientation;
                        List<IMyThrust> allThrusters = new List<IMyThrust>();
                        Parent.GridTerminalSystem.GetBlocksOfType(allThrusters,
                        block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                        foreach (var thruster in allThrusters)
                        {
                            blockOrientation = RC.Orientation.TransformDirectionInverse(thruster.Orientation.Forward);
                            if (blockOrientation == Base6Directions.Direction.Down)
                                Thrusters.Add(thruster);
                        }
                        ReleaseControl();
                    }
                }
                public void ReleaseControl()
                {
                    foreach (var thruster in Thrusters)
                        thruster.ThrustOverride = 0;
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!StMachine.MoveNext())
                                StMachine = UpdateIterator();
                        }
                        else if (EnableState)
                        {
                            ReleaseControl();
                            StMachine = UpdateIterator();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                    {
                        Time += dT;
                        if (Camera.Ready && Camera.DataAge == 0)
                        {
                            EmpiricDescentVeloc = (AltitudePrev - Camera.Altitude) / Time;
                            Time = 0;
                            AltitudePrev = Altitude = Camera.Altitude;
                            AltitudeAge = 0;
                        }
                        else
                            IntegrateAltitude();
                        yield return true;
                    }
                    CycleNum = 0;

                    Time += dT;
                    if (Camera.Ready && Camera.DataAge == 0)
                    {
                        EmpiricDescentVeloc = (AltitudePrev - Camera.Altitude) / Time;
                        Time = 0;
                        AltitudePrev = Altitude = Camera.Altitude;
                        AltitudeAge = 0;
                    }
                    else if (AltitudeAge > Camera.SkipCycles * 3)
                    {
                        double alt;
                        RC.TryGetPlanetElevation(MyPlanetElevation.Surface, out alt);
                        alt -= RC.CubeGrid.WorldVolume.Radius;
                        EmpiricDescentVeloc = (AltitudePrev - alt) / Time;
                        Time = 0;
                        AltitudePrev = Altitude = alt;
                        AltitudeAge = 0;
                    }
                    else
                        IntegrateAltitude();
                    double discriminantOver4 = -1;
                    double fallVelocity = 0;
                    if (Gravity.Magnitude > 0)
                    {
                        double totalThrustForce = 0;
                        foreach (var thruster in Thrusters)
                            if (thruster.IsWorking)
                                totalThrustForce += thruster.MaxEffectiveThrust;
                        double liftThrust = totalThrustForce / Mass.Masses.PhysicalMass - Gravity.Magnitude;
                        EnoughThrust = liftThrust > 0;
                        if (Dynamics.Velocity.Down > SafeFallVelocity)
                            fallVelocity = Dynamics.Velocity.Down * 0.3 + EmpiricDescentVeloc * 0.7;
                        else
                            fallVelocity = Dynamics.Velocity.Down;
                        discriminantOver4 = fallVelocity * fallVelocity - liftThrust * 2 * Altitude;
                    }
                    if ((discriminantOver4 >= 0 && fallVelocity > SafeFallVelocity) || (UserInput.CSpacebar > 0 && UseJumpJets))
                        foreach (var thruster in Thrusters)
                            thruster.ThrustOverridePercentage = 1;
                    else if (UserInput != null)
                        foreach (var thruster in Thrusters)
                            thruster.ThrustOverride = 0.01f;
                }
                void IntegrateAltitude()
                {
                    Altitude += Dynamics.Velocity.Up * dT;
                    ++AltitudeAge;
                }
            }

            class GyroStabilisationAssisting : BaseClass
            {
                readonly List<IMyGyro> Gyros;
                readonly GravityWatcher Gravity;
                readonly OrientationComputer OrientationPrime;
                readonly OrientationComputer OrientationSecond;
                readonly CameraScheduler Camera;
                readonly GridDynamicsWatcher Dynamics;
                IInputData UserInput;
                readonly PID PitchPID;
                readonly PID RollPID;
                Vector3D AnglesPrev;
                int CycleNum;
                IEnumerator<bool> StMachine;
                double Time;
                bool EnableState;

                public bool Enabled;
                new public bool Ready { get { return (this as BaseClass).Ready && Gyros.Count > 0; } }
                public bool IsAllGyrosWorking { get; set; }
                public int SkipCycles;
                public double dT = 0.016;
                public double MinGyroPower = 0.2;
                public double NaturalPitchDegree;
                public double VehicleTurnRadiusRight;
                public double VehicleTurnRadiusLeft;
                public double CoMToWheelsBaseAngle;
                public GyroStabilisationAssisting(Program parent, GravityWatcher gravity,
                OrientationComputer orientation, CameraScheduler camera, GridDynamicsWatcher dynamics,
                int skipCycles = 0, int skipOffset = 0) : base(parent)
                {
                    Gravity = gravity;
                    OrientationPrime = orientation;
                    Camera = camera;
                    Dynamics = dynamics;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    Gyros = new List<IMyGyro>();
                    OrientationSecond = new OrientationComputer(parent);
                    PitchPID = new PID(3, 0, 0.33);
                    RollPID = new PID(3, 0, 0.33);
                    StMachine = UpdateIterator();
                    IsAllGyrosWorking = true;
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    Gyros.Clear();
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    if (RC != null)
                    {
                        Parent.GridTerminalSystem.GetBlocksOfType(Gyros,
                        block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                        OrientationSecond.RC = RC;
                    }
                }
                public void ReleaseControl()
                {
                    foreach (var gyro in Gyros)
                    {
                        gyro.GyroOverride = false;
                        gyro.GyroPower = 100;
                    }
                }
                public void SetInputSource(IInputData source)
                {
                    UserInput = source;
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!StMachine.MoveNext())
                                StMachine = UpdateIterator();
                        }
                        else if (EnableState)
                        {
                            ReleaseControl();
                            StMachine = UpdateIterator();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateIterator()
                {
                    int temp = SkipCycles - 1;
                    for (; CycleNum < temp; ++CycleNum)
                    {
                        Time += dT;
                        yield return true;
                    }
                    CycleNum = 0;

                    Time += dT;
                    if (Gravity.Magnitude == 0) yield break;
                    Vector3D angles = OrientationPrime.RollYawPitch;
                    if (!Vector3D.IsZero(Camera.SurfaceTilt))
                    {
                        Vector3D unused = Gravity.Direction;
                        Vector3D tempv = Vector3D.Reject(Gravity.Direction, Vector3D.Normalize(Camera.SurfaceTilt));
                        if (!Vector3D.IsZero(Camera.SurfaceBank))
                            tempv = Vector3D.Reject(tempv, Vector3D.Normalize(Camera.SurfaceBank));
                        OrientationSecond.Update(tempv, true);
                        angles = OrientationSecond.RollYawPitch;
                        if (Math.Abs(angles.X) < 0.05236) // 3 degree
                            angles.X = 0;
                        if (Math.Abs(angles.Z) < 0.05236)
                            angles.Z = 0;
                    }
                    else if (Dynamics.Velocity.Down > 3)
                    {
                        double vLen = Gravity.Magnitude * 0.364;// tan(20 deg)
                        Vector3D tempv = Vector3D.Normalize(Dynamics.Velocity.Vector) * vLen + Gravity.Direction;
                        OrientationSecond.Update(tempv, true);
                        angles = OrientationSecond.RollYawPitch;
                    }

                    angles.Z -= MathHelperD.ToRadians(NaturalPitchDegree);
                    angles.X = -angles.X;
                    angles.Y = 0;
                    if (UserInput.AD != 0)
                    {
                        if (Dynamics.Velocity.VerticalAbs > 0.5)
                        {
                            angles.Y = UserInput.AD * 0.618;
                            if (Dynamics.Velocity.Backward >= 0.1)
                                angles.Y = -angles.Y;
                        }
                        else
                        {
                            if (UserInput.AD > 0 && VehicleTurnRadiusRight > 0)
                            {
                                double safeTurnRadiusRight = Math.Pow(Dynamics.Velocity.LateralAbs, 1.5) / (Gravity.Magnitude * Math.Cos(MathHelperD.Clamp(CoMToWheelsBaseAngle - OrientationPrime.Roll, 1e-6, MathHelperD.Pi / 2.1)));
                                angles.Y = Dynamics.Velocity.Forward / Math.Max(VehicleTurnRadiusRight, safeTurnRadiusRight) * UserInput.AD;
                            }
                            else if (UserInput.AD < 0 && VehicleTurnRadiusLeft < 0)
                            {
                                double safeTurnRadiusLeft = Math.Pow(Dynamics.Velocity.LateralAbs, 1.5) / (Gravity.Magnitude * Math.Cos(MathHelperD.Clamp(CoMToWheelsBaseAngle + OrientationPrime.Roll, 1e-6, MathHelperD.Pi / 2.1)));
                                angles.Y = Dynamics.Velocity.Forward / Math.Min(VehicleTurnRadiusLeft, -safeTurnRadiusLeft) * -UserInput.AD;
                            }
                        }
                    }
                    Vector3D PIDSignal = Vector3D.Zero;
                    bool blackScienceEquation = (angles - AnglesPrev).AbsMax() >= angles.AbsMax() / 5 || angles.Y != 0;
                    if (blackScienceEquation)
                    {
                        AnglesPrev = angles;
                        PIDSignal.Y = angles.Y;
                        PIDSignal.X = RollPID.GetSignal(angles.X, Time);
                        PIDSignal.Z = PitchPID.GetSignal(angles.Z, Time);
                    }
                    if (SkipCycles > 0)
                    {
                        yield return true;
                        Time += dT;
                    }
                    double gyroPower = MathHelperD.Clamp(angles.AbsMax() / MathHelperD.PiOver4, MinGyroPower, 1.0);
                    Base6Directions.Direction gyroOrientation;
                    int workingGyrosCount = 0;
                    foreach (var gyro in Gyros)
                    {
                        if (gyro.IsWorking)
                        {
                            ++workingGyrosCount;
                            if (blackScienceEquation)
                            {
                                gyro.GyroOverride = true;
                                gyro.GyroPower = (float)gyroPower;
                                gyroOrientation = RC.Orientation.TransformDirectionInverse(gyro.Orientation.Up);
                                switch (gyroOrientation)
                                {
                                    case Base6Directions.Direction.Up: gyro.Yaw = (float)PIDSignal.Y; break;
                                    case Base6Directions.Direction.Down: gyro.Yaw = -(float)PIDSignal.Y; break;
                                    case Base6Directions.Direction.Forward: gyro.Yaw = -(float)PIDSignal.X; break;
                                    case Base6Directions.Direction.Backward: gyro.Yaw = (float)PIDSignal.X; break;
                                    case Base6Directions.Direction.Left: gyro.Yaw = -(float)PIDSignal.Z; break;
                                    case Base6Directions.Direction.Right: gyro.Yaw = (float)PIDSignal.Z; break;
                                }
                                gyroOrientation = RC.Orientation.TransformDirectionInverse(gyro.Orientation.Left);
                                switch (gyroOrientation)
                                {
                                    case Base6Directions.Direction.Up: gyro.Pitch = -(float)PIDSignal.Y; break;
                                    case Base6Directions.Direction.Down: gyro.Pitch = (float)PIDSignal.Y; break;
                                    case Base6Directions.Direction.Forward: gyro.Pitch = (float)PIDSignal.X; break;
                                    case Base6Directions.Direction.Backward: gyro.Pitch = -(float)PIDSignal.X; break;
                                    case Base6Directions.Direction.Left: gyro.Pitch = (float)PIDSignal.Z; break;
                                    case Base6Directions.Direction.Right: gyro.Pitch = -(float)PIDSignal.Z; break;
                                }
                                gyroOrientation = RC.Orientation.TransformDirectionInverse(gyro.Orientation.Forward);
                                switch (gyroOrientation)
                                {
                                    case Base6Directions.Direction.Up: gyro.Roll = -(float)PIDSignal.Y; break;
                                    case Base6Directions.Direction.Down: gyro.Roll = (float)PIDSignal.Y; break;
                                    case Base6Directions.Direction.Forward: gyro.Roll = (float)PIDSignal.X; break;
                                    case Base6Directions.Direction.Backward: gyro.Roll = -(float)PIDSignal.X; break;
                                    case Base6Directions.Direction.Left: gyro.Roll = (float)PIDSignal.Z; break;
                                    case Base6Directions.Direction.Right: gyro.Roll = -(float)PIDSignal.Z; break;
                                }
                            }
                        }
                    }
                    IsAllGyrosWorking = workingGyrosCount == Gyros.Count;
                }
            }

            class SmartDampenersOverride : BaseClass
            {
                readonly List<IMyThrust> ForwardThrusters;
                readonly List<IMyThrust> BackwardThrusters;
                readonly List<IMyThrust> RightwardThrusters;
                readonly List<IMyThrust> LeftwardThrusters;
                readonly GridDynamicsWatcher Dynamics;
                IInputData UserInput;
                int CycleNum;
                IEnumerator<bool> StMachine;
                bool EnableState;

                public bool Enabled;
                new public bool Ready
                {
                    get
                    {
                        return (this as BaseClass).Ready &&
                        (ForwardThrusters.Count + BackwardThrusters.Count + RightwardThrusters.Count + LeftwardThrusters.Count) > 0;
                    }
                }
                public int SkipCycles;
                public SmartDampenersOverride(Program parent, GridDynamicsWatcher dynamics,
                int skipCycles = 0, int skipOffset = 0) : base(parent)
                {
                    Dynamics = dynamics;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    ForwardThrusters = new List<IMyThrust>();
                    BackwardThrusters = new List<IMyThrust>();
                    RightwardThrusters = new List<IMyThrust>();
                    LeftwardThrusters = new List<IMyThrust>();
                    StMachine = UpdateIterator();
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    ForwardThrusters.Clear();
                    BackwardThrusters.Clear();
                    RightwardThrusters.Clear();
                    LeftwardThrusters.Clear();
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    if (RC != null)
                    {
                        Base6Directions.Direction blockOrientation;
                        List<IMyThrust> allThrusters = new List<IMyThrust>();
                        Parent.GridTerminalSystem.GetBlocksOfType(allThrusters,
                        block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                        foreach (var thruster in allThrusters)
                        {
                            blockOrientation = RC.Orientation.TransformDirectionInverse(thruster.Orientation.Forward);
                            switch (blockOrientation)
                            {
                                case Base6Directions.Direction.Backward:
                                    ForwardThrusters.Add(thruster);
                                    break;
                                case Base6Directions.Direction.Forward:
                                    BackwardThrusters.Add(thruster);
                                    break;
                                case Base6Directions.Direction.Right:
                                    LeftwardThrusters.Add(thruster);
                                    break;
                                case Base6Directions.Direction.Left:
                                    RightwardThrusters.Add(thruster);
                                    break;
                            }
                        }
                    }
                }
                public void ReleaseControl()
                {
                    foreach (var thruster in ForwardThrusters)
                        thruster.ThrustOverride = 0;
                    foreach (var thruster in BackwardThrusters)
                        thruster.ThrustOverride = 0;
                    foreach (var thruster in LeftwardThrusters)
                        thruster.ThrustOverride = 0;
                    foreach (var thruster in RightwardThrusters)
                        thruster.ThrustOverride = 0;
                }
                public void SetInputSource(IInputData source)
                {
                    UserInput = source;
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!StMachine.MoveNext())
                                StMachine = UpdateIterator();
                        }
                        else if (EnableState)
                        {
                            ReleaseControl();
                            StMachine = UpdateIterator();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;

                    if ((RC.DampenersOverride && (UserInput.WS >= 0 || Dynamics.Velocity.Backward > 1)) || (UserInput.WS != 0 && UserInput.CSpacebar < 0))
                        foreach (var thruster in ForwardThrusters)
                            thruster.ThrustOverride = 0;
                    else
                        foreach (var thruster in ForwardThrusters)
                            thruster.ThrustOverride = 0.01f;

                    if ((RC.DampenersOverride && (UserInput.WS <= 0 || Dynamics.Velocity.Forward > 1)) || (UserInput.WS != 0 && UserInput.CSpacebar < 0))
                        foreach (var thruster in BackwardThrusters)
                            thruster.ThrustOverride = 0;
                    else
                        foreach (var thruster in BackwardThrusters)
                            thruster.ThrustOverride = 0.01f;

                    if (RC.DampenersOverride && !(UserInput.AD > 0 && Dynamics.Velocity.Right > 1) && Dynamics.Velocity.SideAbs > 1)
                    {
                        if (UserInput.AD < 0 && Dynamics.Velocity.Left > 1)
                            foreach (var thruster in RightwardThrusters)
                                thruster.ThrustOverridePercentage = 1;
                        else
                            foreach (var thruster in RightwardThrusters)
                                thruster.ThrustOverride = 0;
                    }
                    else
                        foreach (var thruster in RightwardThrusters)
                            thruster.ThrustOverride = 0.01f;

                    if (RC.DampenersOverride && !(UserInput.AD < 0 && Dynamics.Velocity.Left > 1) && Dynamics.Velocity.SideAbs > 1)
                    {
                        if (UserInput.AD > 0 && Dynamics.Velocity.Right > 1)
                            foreach (var thruster in LeftwardThrusters)
                                thruster.ThrustOverridePercentage = 1;
                        else
                            foreach (var thruster in LeftwardThrusters)
                                thruster.ThrustOverride = 0;
                    }
                    else
                        foreach (var thruster in LeftwardThrusters)
                            thruster.ThrustOverride = 0.01f;
                }
            }

            class LightsManager : BaseClass
            {
                readonly List<IMyLightingBlock> StopLights;
                readonly List<IMyLightingBlock> TurnLights;
                readonly GridDynamicsWatcher Dynamics;
                IInputData UserInput;
                int CycleNum;
                IEnumerator<bool> StMachine;
                bool EnableState;
                readonly string StopLightsGroupName;
                readonly string TurnLightsGroupName;

                public bool Enabled;
                new public bool Ready { get { return (this as BaseClass).Ready && (StopLights.Count + TurnLights.Count) > 0; } }
                public int SkipCycles;
                public LightsManager(Program parent, GridDynamicsWatcher dynamics, string stopLightGrpNm,
                string turnLightGrpNm, int skipCycles = 0, int skipOffset = 0) : base(parent)
                {
                    Dynamics = dynamics;
                    StopLightsGroupName = stopLightGrpNm;
                    TurnLightsGroupName = turnLightGrpNm;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StopLights = new List<IMyLightingBlock>();
                    TurnLights = new List<IMyLightingBlock>();
                    StMachine = UpdateIterator();
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    StopLights.Clear();
                    TurnLights.Clear();
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    if (RC != null)
                    {
                        IMyBlockGroup lightsGroup = Parent.GridTerminalSystem.GetBlockGroupWithName(StopLightsGroupName);
                        if (lightsGroup != null)
                            lightsGroup.GetBlocksOfType(StopLights, block => block.IsSameConstructAs(Parent.Me));

                        lightsGroup = Parent.GridTerminalSystem.GetBlockGroupWithName(TurnLightsGroupName);
                        if (lightsGroup != null)
                            lightsGroup.GetBlocksOfType(TurnLights, block => block.IsSameConstructAs(Parent.Me));
                    }
                }
                public void ReleaseControl()
                {
                    foreach (var light in StopLights)
                        light.Enabled = true;
                    foreach (var light in TurnLights)
                        light.Enabled = true;
                }
                public void SetInputSource(IInputData source)
                {
                    UserInput = source;
                }
                public void Update()
                {
                    if (Ready)
                    {
                        if (Enabled)
                        {
                            if (!StMachine.MoveNext())
                                StMachine = UpdateIterator();
                        }
                        else if (EnableState)
                        {
                            ReleaseControl();
                            StMachine = UpdateIterator();
                        }
                        EnableState = Enabled;
                    }
                }
                IEnumerator<bool> UpdateIterator()
                {
                    int temp = SkipCycles - 1;
                    for (; CycleNum < temp; ++CycleNum)
                        yield return true;
                    CycleNum = 0;

                    foreach (var light in StopLights)
                    {
                        if ((Dynamics.Velocity.Forward > 0.4 && Dynamics.Acceleration.AvrEMA < 0 && UserInput.WS >= 0) || UserInput.CSpacebar > 0)
                        {
                            light.Radius = 2f;
                            light.Intensity = 5f;
                            light.Falloff = 1.3f;
                            light.Color = Color.Red;
                        }
                        else if (Dynamics.Velocity.Backward > 0.5)
                        {
                            light.Radius = 5f;
                            light.Intensity = 5f;
                            light.Falloff = 1.3f;
                            light.Color = Color.White;
                        }
                        else
                        {
                            light.Radius = 1f;
                            light.Intensity = 1f;
                            light.Falloff = 0;
                            light.Color = Color.DarkRed;
                        }
                    }
                    if (SkipCycles > 0) yield return true;
                    foreach (var light in TurnLights)
                    {
                        if (light.CustomData.Equals("LEFT", StringComparison.OrdinalIgnoreCase))
                        {
                            if (UserInput.AD < 0)
                                light.Enabled = true;
                            else
                                light.Enabled = false;
                        }
                        else if (light.CustomData.Equals("RIGHT", StringComparison.OrdinalIgnoreCase))
                        {
                            if (UserInput.AD > 0)
                                light.Enabled = true;
                            else
                                light.Enabled = false;
                        }
                    }
                }
            }

            class DisplayScheduler
            {
                readonly Program Parent;
                readonly List<IMyTextSurface> LCDs;
                int CycleNum;
                int PrintCounter;
                IEnumerator<bool> StMachine;
                readonly string Keyword;
                readonly StringBuilder Str;
                readonly CallBackFunk CallBack;

                public delegate StringBuilder CallBackFunk(int i);
                public bool Ready { get; set; }
                public int SkipCycles;
                public string Caption;

                public DisplayScheduler(Program parent, CallBackFunk callBack, string keyword, int skipCycles = 0, int skipOffset = 0)
                {
                    Parent = parent;
                    CallBack = callBack;
                    Keyword = keyword;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    LCDs = new List<IMyTextSurface>();
                    Str = new StringBuilder();
                    StMachine = UpdateIterator();
                }
                public void DetectBlocks()
                {
                    LCDs.Clear();
                    List<IMyTerminalBlock> lcdHosts = new List<IMyTerminalBlock>();
                    Parent.GridTerminalSystem.GetBlocksOfType(lcdHosts, block => block as IMyTextSurfaceProvider != null && block.IsSameConstructAs(Parent.Me));
                    List<string> lines = new List<string>();
                    IMyTextSurface lcd;
                    foreach (var block in lcdHosts)
                    {
                        if (block.CustomData.Length > 0)
                        {
                            lines.Clear();
                            new StringSegment(block.CustomData).GetLines(lines);
                            foreach (var line in lines)
                            {
                                if (line.Contains(Keyword))
                                {
                                    if (block as IMyTextSurface != null)
                                        lcd = block as IMyTextSurface;
                                    else
                                    {
                                        int i = 0;
                                        int.TryParse(line.Replace(Keyword, ""), out i);
                                        IMyTextSurfaceProvider t_sp = block as IMyTextSurfaceProvider;
                                        i = Math.Max(0, Math.Min(i, t_sp.SurfaceCount));
                                        lcd = t_sp.GetSurface(i);
                                    }
                                    lcd.ContentType = ContentType.TEXT_AND_IMAGE;
                                    LCDs.Add(lcd);
                                }
                            }
                        }
                    }
                    Ready = LCDs.Count > 0;
                }

                public void ForcePrint(StringBuilder text = null)
                {
                    if (Ready)
                    {
                        Str.Clear();
                        Str.Append(Caption);
                        ++PrintCounter;
                        switch (PrintCounter % 4)
                        {
                            case 0: Str.Append("--"); break;
                            case 1: Str.Append("\\"); break;
                            case 2: Str.Append("|"); break;
                            case 3: Str.Append("/"); break;
                        }
                        Str.Append("\n_________________________________________________________\n");
                        if (text != null)
                            Str.Append(text);
                        else
                            Str.Append(CallBack(PrintCounter));
                        foreach (var lcd in LCDs)
                            lcd.WriteText(Str);
                    }
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    while (CycleNum < SkipCycles)
                    {
                        ++CycleNum;
                        yield return true;
                    }
                    CycleNum = 0;
                    ForcePrint();
                }
            }

            class AutoHandbrakesSubroutine : BaseClass
            {
                IInputData UserInput;
                int CycleNum;
                IEnumerator<bool> StMachine;
                bool WasManned;

                public int SkipCycles;
                public HandbrakeMode Mode;
                public AutoHandbrakesSubroutine(Program parent, int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                }
                public void SetInputSource(IInputData source)
                {
                    UserInput = source;
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;
                    if (WasManned && !UserInput.Manning && (Mode == HandbrakeMode.Auto || Mode == HandbrakeMode.Semi))
                        RC.HandBrake = true;
                    if (!WasManned && UserInput.Manning && Mode == HandbrakeMode.Auto)
                        RC.HandBrake = false;
                    WasManned = UserInput.Manning;
                }
            }

            class ArtificialMassSubroutine
            {
                readonly Program Parent;
                readonly List<IMyVirtualMass> VirtualMassBlocks;
                readonly GravityWatcher Gravity;
                int CycleNum;
                IEnumerator<bool> StMachine;

                public bool Ready { get; private set; }
                public int SkipCycles;
                public ArtificialMassSubroutine(Program parent, GravityWatcher gravity, int skipCycles = 0, int skipOffset = 0)
                {
                    Parent = parent;
                    Gravity = gravity;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    VirtualMassBlocks = new List<IMyVirtualMass>();
                    StMachine = UpdateIterator();
                }
                public void DetectBlocks()
                {
                    VirtualMassBlocks.Clear();
                    Parent.GridTerminalSystem.GetBlocksOfType(VirtualMassBlocks,
                    block => block.CubeGrid == Parent.Me.CubeGrid && !Utils.IsIgnore(block));
                    Ready = VirtualMassBlocks.Count > 0;
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;
                    if (Gravity.ArtificialStongerNatural)
                        foreach (var block in VirtualMassBlocks)
                            block.Enabled = true;
                    else
                        foreach (var block in VirtualMassBlocks)
                            block.Enabled = false;
                }
            }

            class GPSTrackerSubroutine : BaseClass
            {
                bool EnableState;
                readonly IInputData UserInput;
                readonly GridDynamicsWatcher Dynamics;
                float ADPrev;
                int WaypointNum;

                public bool Enabled;
                public StringBuilder GPSTrackList { get; }
                public GPSTrackerSubroutine(Program parent, IInputData userInput, GridDynamicsWatcher dynamics) : base(parent)
                {
                    UserInput = userInput;
                    Dynamics = dynamics;
                    GPSTrackList = new StringBuilder();
                }
                public void Update()
                {
                    if (Enabled)
                    {
                        if (!EnableState)
                        {
                            WaypointNum = 1;
                            GPSTrackList.Clear();
                            ADPrev = 1 - UserInput.AD;
                        }
                        if (UserInput.AD != ADPrev && Dynamics.Velocity.VerticalAbs < 1)
                        {
                            GPSTrackList.AppendFormat("GPS:{0} #{1:D3}:{2:F2}:{3:F2}:{4:F2}:\n", RC.CubeGrid.CustomName, WaypointNum,
                            RC.CubeGrid.WorldVolume.Center.X, RC.CubeGrid.WorldVolume.Center.Y, RC.CubeGrid.WorldVolume.Center.Z);
                            ++WaypointNum;
                            ADPrev = UserInput.AD;
                        }
                    }
                    else if (EnableState)
                        GPSTrackList.AppendFormat("GPS:{0} #{1:D3}:{2:F2}:{3:F2}:{4:F2}:\n", RC.CubeGrid.CustomName, WaypointNum,
                        RC.CubeGrid.WorldVolume.Center.X, RC.CubeGrid.WorldVolume.Center.Y, RC.CubeGrid.WorldVolume.Center.Z);
                    EnableState = Enabled;
                }
            }

            class BackupBeeperSubroutine
            {
                readonly Program Parent;
                int CycleNum;
                bool IsPlaying;
                readonly string SoundBlocksName;
                IEnumerator<bool> StMachine;
                readonly GridDynamicsWatcher Dynamics;
                readonly List<IMySoundBlock> SoundBlocks;

                public bool Ready { get; set; }
                public int SkipCycles;
                public BackupBeeperSubroutine(Program parent, GridDynamicsWatcher dynamics, string soundBlocksName,
                int skipCycles = 0, int skipOffset = 0)
                {
                    Parent = parent;
                    Dynamics = dynamics;
                    SoundBlocksName = soundBlocksName;
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                    SoundBlocks = new List<IMySoundBlock>();
                }
                public void DetectBlocks()
                {
                    SoundBlocks.Clear();
                    Parent.GridTerminalSystem.GetBlocksOfType(SoundBlocks, block => block.IsSameConstructAs(Parent.Me) && block.CustomName.Contains(SoundBlocksName));
                    Ready = SoundBlocks.Count > 0;
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    while (CycleNum < SkipCycles)
                    {
                        ++CycleNum;
                        yield return true;
                    }
                    CycleNum = 0;
                    if (Dynamics.Velocity.Backward > 0.5)
                    {
                        if (!IsPlaying)
                        {
                            foreach (var sb in SoundBlocks)
                                sb.Play();
                            IsPlaying = true;
                        }
                    }
                    else
                    {
                        if (IsPlaying)
                        {
                            foreach (var sb in SoundBlocks)
                                sb.Stop();
                            IsPlaying = false;
                        }
                    }
                }
            }

            class ConnectorSubroutine : BaseClass
            {
                int CycleNum;
                IEnumerator<bool> StMachine;
                readonly List<IMyShipConnector> Connectors;

                new public bool Ready { get { return (this as BaseClass).Ready && Connectors.Count > 0; } }
                public int SkipCycles;
                public bool IsConnected { get; private set; }
                public ConnectorSubroutine(Program parent, int skipCycles = 0, int skipOffset = 0)
                : base(parent)
                {
                    SkipCycles = skipCycles;
                    CycleNum = skipOffset;
                    StMachine = UpdateIterator();
                    Connectors = new List<IMyShipConnector>();
                }
                public void DetectBlocks(IMyRemoteControl rc = null)
                {
                    if (rc != null)
                        RC = rc;
                    else
                        ForceDetectRemoteControl();
                    Connectors.Clear();
                    Parent.GridTerminalSystem.GetBlocksOfType(Connectors,
                    block => block.IsSameConstructAs(Parent.Me) && !Utils.IsIgnore(block));
                }
                public void Update()
                {
                    if (Ready)
                        if (!StMachine.MoveNext())
                            StMachine = UpdateIterator();
                }
                IEnumerator<bool> UpdateIterator()
                {
                    for (; CycleNum < SkipCycles; ++CycleNum)
                        yield return true;
                    CycleNum = 0;
                    IsConnected = false;
                    foreach (var con in Connectors)
                        if (con.Status == MyShipConnectorStatus.Connected)
                        {
                            IsConnected = true;
                            break;
                        }
                }
            }

            readonly UserInputWatcher UserInput;
            readonly AutopilotDriver Autopilot;
            readonly CruiseControlUnit CruiseControl;
            readonly GridDynamicsWatcher VehicleDynamics;
            readonly GridMassWatcher VehicleMass;
            readonly GravityWatcher Gravity;
            readonly OrientationComputer Orientation;
            readonly CameraScheduler Cameras;
            readonly TractionControlSystem TCS;
            readonly FallDampeningSystem FDS;
            readonly GyroStabilisationAssisting GSA;
            readonly SmartDampenersOverride SDO;
            readonly LightsManager Lights;
            readonly DisplayScheduler NavgtnDisplays;
            readonly DisplayScheduler StatusDisplays;
            readonly AutoHandbrakesSubroutine AutoBrakes;
            readonly ArtificialMassSubroutine ArtMass;
            readonly GPSTrackerSubroutine GPSTracker;
            readonly BackupBeeperSubroutine BackupBeeper;
            readonly ConnectorSubroutine ConnectorSubrtn;
            readonly RuntimeProfiler Profiler;

            IEnumerator<bool> BootStMachine;
            readonly Program Parent;
            bool Booted;
            bool Busy;
            bool ShowGPSList;
            readonly StringBuilder StrN;
            readonly StringBuilder StrS;

            public double MaxForwardAcceleration { get { return _MaxForwardAccel; } set { _MaxForwardAccel = TCS.MaxForwardAcceleration = value; } }
            public double MaxBackwardAcceleration { get { return _MaxBackwardAccel; } set { _MaxBackwardAccel = TCS.MaxBackwardAcceleration = value; } }
            public double MaxPowerConsumption { get { return _MaxPowerConsumption; } set { _MaxPowerConsumption = TCS.PowerLimit = value; } }
            public double NaturalPitchDegree { get { return _NaturalPitchDegree; } set { _NaturalPitchDegree = GSA.NaturalPitchDegree = VehicleDynamics.NaturalPitchDegree = value; } }
            public double SafeFallVelocity { get { return _SafeFallVelocity; } set { _SafeFallVelocity = FDS.SafeFallVelocity = value; } }
            public double LeanToSurfaceDistance { get { return _LeanToSurfaceDistance; } set { _LeanToSurfaceDistance = Cameras.CalcSurfaceAfter = value; } }
            public double MaxSteerAngleDegree { get { return _MaxSteerAngleDegree; } set { _MaxSteerAngleDegree = TCS.MaxSteerAngleDegree = value; } }
            public double ForwardSpeedLimitKPH { get { return _FrwrdSpeedLimitKPH; } set { _FrwrdSpeedLimitKPH = TCS.ForwardSpeedLimitKPH = value; } }
            public double BackwardSpeedLimitKPH { get { return _BckwrdSpeedLimitKPH; } set { _BckwrdSpeedLimitKPH = TCS.BackwardSpeedLimitKPH = value; } }
            public double SuspensionHeightOffset { get { return _SusHeightOffset; } set { _SusHeightOffset = TCS.HeightOffset = value; } }
            public double SuspensionStrenght { get { return _SusStrenght; } set { _SusStrenght = TCS.Strenght = value; } }
            public double SuspensionValueChangeRate { get { return _SusValueChangeRate; } set { _SusValueChangeRate = TCS.ValueChangeRate = value; } }
            public double AutopilotAccuracy { get { return _AutopilotAccuracy; } set { _AutopilotAccuracy = Autopilot.Accuracy = value; } }
            public double AutopilotWait { get { return _AutopilotWait; } set { _AutopilotWait = Autopilot.WaitTimeSec = value; } }
            public double AutopilotSpeedLimitKMP { get { return _AutopilotSpeedLimitKMP; } set { _AutopilotSpeedLimitKMP = value; } }
            public bool UseJumpJets { get { return _UseJumpJets; } set { _UseJumpJets = FDS.UseJumpJets = value; } }
            public bool UseAdaptiveSteering { get { return _UseAdaptiveSteering; } set { _UseAdaptiveSteering = TCS.UseAdaptiveSteering = value; } }
            public bool UseAckermannSteering { get { return _UseAckermannSteering; } set { _UseAckermannSteering = TCS.UseAckermannSteering = value; } }
            public bool UseDASAirShock { get { return _UseDASAirShock; } set { _UseDASAirShock = TCS.UseDASAirShock = value; } }
            public bool UseHillDescentControl { get { return _UseHillDescentControl; } set { _UseHillDescentControl = value; TCS.UseHillDescentControl = Autopilot.Enabled || value; } }
            public bool UseGSA { get { return _UseGSA; } set { _UseGSA = GSA.Enabled = value; } }
            public bool UseTCS { get { return _UseTCS; } set { _UseTCS = TCS.Enabled = value; } }
            public bool UseFDS { get { return _UseFDS; } set { _UseFDS = FDS.Enabled = value; } }
            public bool UseSDO { get { return _UseSDO; } set { _UseSDO = SDO.Enabled = value; } }
            public bool UseLights { get { return _UseLights; } set { _UseLights = Lights.Enabled = value; } }
            public bool Run;
            public double Odometer;

            public DriverAssistingSystem(Program parentProgram)
            {
                Parent = parentProgram;

                UserInput = new UserInputWatcher(Parent);

                VehicleDynamics = new GridDynamicsWatcher(Parent);
                VehicleMass = new GridMassWatcher(Parent, 9, 9);
                VehicleMass.SetIgnorArray(new float[] {
100,// player
420,// l.g. 1x1 wheel
105,// s.g. 1x1
590,// l.g. 3x3
205,// s.g. 3x3
760,// l.g. 5x5
310 // s.g. 5x5
});
                Gravity = new GravityWatcher(Parent, 59, 56);
                Orientation = new OrientationComputer(Parent, 2, 2);
                Cameras = new CameraScheduler(Parent, VehicleDynamics, 2, 2);
                TCS = new TractionControlSystem(Parent, VehicleDynamics, Gravity, Orientation, Cameras,
                _SubgridWheelsGroupName, 1, 1, 7, 4, 7, 1, 7, 0, 9, 5, 7, 5, 9, 7, 59, 53);
                FDS = new FallDampeningSystem(Parent, VehicleDynamics, Gravity, VehicleMass, Cameras, UserInput, 3);
                GSA = new GyroStabilisationAssisting(Parent, Gravity, Orientation, Cameras, VehicleDynamics, 3, 2);
                SDO = new SmartDampenersOverride(Parent, VehicleDynamics);
                Lights = new LightsManager(Parent, VehicleDynamics, _StopLightsGroupName, _TurnLightsGroupName, 9, 4);
                Autopilot = new AutopilotDriver(Parent, _AutopilotTimerName, 5);
                CruiseControl = new CruiseControlUnit(Parent, UserInput, VehicleDynamics, Orientation, 5);
                AutoBrakes = new AutoHandbrakesSubroutine(Parent, 9, 9);
                ArtMass = new ArtificialMassSubroutine(Parent, Gravity, 99, 91);
                GPSTracker = new GPSTrackerSubroutine(Parent, UserInput, VehicleDynamics);
                BackupBeeper = new BackupBeeperSubroutine(Parent, VehicleDynamics, _BackupBeeperName, 9, 8);
                ConnectorSubrtn = new ConnectorSubroutine(Parent);

                NavgtnDisplays = new DisplayScheduler(Parent, ComposeNavigationData, "DAS_navigation", 14, 2);
                StatusDisplays = new DisplayScheduler(Parent, ComposeStatusData, "DAS_status", 29, 8);
                Profiler = new RuntimeProfiler(Parent, 14)
                {
                    Caption = "DAS - Driver Assisting System  "
                };

                Run = true;
                Load();
                VehicleDynamics.NaturalPitchDegree = _NaturalPitchDegree;
                TCS.MaxForwardAcceleration = _MaxForwardAccel;
                TCS.MaxBackwardAcceleration = _MaxBackwardAccel;
                TCS.PowerLimit = _MaxPowerConsumption;
                TCS.AckFocal = _AckermannFocalPoint;
                TCS.UseAdaptiveSteering = _UseAdaptiveSteering;
                TCS.UseAckermannSteering = _UseAckermannSteering;
                TCS.UseDASAirShock = _UseDASAirShock;
                TCS.UseHillDescentControl = _UseHillDescentControl;
                TCS.ForwardSpeedLimitKPH = _FrwrdSpeedLimitKPH;
                TCS.BackwardSpeedLimitKPH = _BckwrdSpeedLimitKPH;
                TCS.MaxSteerAngleDegree = _MaxSteerAngleDegree;
                TCS.HeightOffset = _SusHeightOffset;
                TCS.Strenght = _SusStrenght;
                TCS.ValueChangeRate = _SusValueChangeRate;
                TCS.Enabled = _UseTCS;
                FDS.SafeFallVelocity = _SafeFallVelocity;
                FDS.UseJumpJets = _UseJumpJets;
                FDS.Enabled = _UseFDS;
                GSA.NaturalPitchDegree = _NaturalPitchDegree;
                GSA.Enabled = _UseGSA;
                SDO.Enabled = _UseSDO;
                Lights.Enabled = _UseLights;
                Cameras.CalcSurfaceAfter = _LeanToSurfaceDistance;
                AutoBrakes.Mode = _HandbrakeMode;
                Autopilot.WaitTimeSec = _AutopilotWait;
                Autopilot.Accuracy = _AutopilotAccuracy;
                if (Autopilot.Enabled)
                    SetAutopilotControl();
                else
                    SetUserControl();

                StrN = new StringBuilder();
                StrS = new StringBuilder();

                BootStMachine = Boot();
                Booted = false;
                if (Run)
                    Start();
            }

            public void Save()
            {
                MyIni config = new MyIni();
                config.Set("General", "Run", Run);
                config.Set("General", "JumpJets", _UseJumpJets);
                config.Set("General", "HDC", _UseHillDescentControl);
                config.Set("General", "Odometer", Odometer);

                config.Set("Autopilot", "Enabled", Autopilot.Enabled);
                config.Set("Autopilot", "WaitTime", _AutopilotWait);
                config.Set("Autopilot", "Accuracy", _AutopilotAccuracy);
                config.Set("Autopilot", "WaypointIndex", Autopilot.WaypointIndx);
                config.Set("Autopilot", "WaypointSelector", Autopilot.WaypointSelector);
                config.Set("Autopilot", "DriveMode", (int)Autopilot.DriveMode);

                Parent.Storage = config.ToString();
            }

            public void Start()
            {
                Busy = true;
                Run = true;
                Parent.Runtime.UpdateFrequency = UpdateFrequency.Update1;
            }

            public void Stop()
            {
                Parent.Runtime.UpdateFrequency = UpdateFrequency.None;
                Run = false;
                ReleaseAllOverrides();
                NavgtnDisplays.ForcePrint(ComposeNavigationData());
                StatusDisplays.ForcePrint(ComposeStatusData());

            }

            public void Update(UpdateType updateSource)
            {
                if (Booted)
                {
                    double dT = 0;
                    if (updateSource.HasFlag(UpdateType.Update1))
                        dT = 0.016;
                    else if (updateSource.HasFlag(UpdateType.Update10))
                        dT = 0.16;
                    else if (updateSource.HasFlag(UpdateType.Update100))
                        dT = 1.6;
                    FDS.dT = GSA.dT = Cameras.dT = Autopilot.dT = VehicleDynamics.dT = dT;
                    try
                    {
                        ConnectorSubrtn.Update();
                        if (!ConnectorSubrtn.IsConnected)
                        {
                            Gravity.Update();
                            VehicleMass.Update();
                            if (VehicleMass.BaseMassChanged)
                            {
                                Booted = false;
                                return;
                            }
                            if (VehicleMass.PhysicalMassChanged)
                            {
                                TCS.UpdateSuspensionCofiguration();
                                TCS.UpdateSprungWeight(VehicleMass.Masses.PhysicalMass);
                                RelayTCSDataToGSA();
                            }
                            if (Gravity.MagnitudeChanged)
                            {
                                TCS.UpdateSprungWeight(VehicleMass.Masses.PhysicalMass);
                                RelayTCSDataToGSA();
                            }
                            VehicleDynamics.Update();
                            UserInput.Update();
                            Autopilot.Update();
                            CruiseControl.Update();
                            AutoBrakes.Update();
                            TCS.Update();
                            ArtMass.Update();
                            if (Busy)
                            {
                                Cameras.SkipCycles = 2 + (100 - (int)VehicleDynamics.Velocity.Scalar) / 14;
                                Cameras.Update();
                                Orientation.Update(Gravity.Direction);
                                GSA.Update();
                                FDS.Update();
                                SDO.Update();
                                Lights.Update();
                                GPSTracker.Update();
                                BackupBeeper.Update();
                            }
                            if (Autopilot.Enabled && !Autopilot.Busy)
                            {
                                Autopilot.Enabled = false;
                                SetUserControl();
                            }
                            if (CruiseControl.Enabled)
                            {
                                if (CruiseControl.Busy)
                                    TCS.ForwardSpeedLimitKPH = TCS.BackwardSpeedLimitKPH = CruiseControl.SpeedKPH;
                                else
                                {
                                    CruiseControl.Enabled = false;
                                    SetUserControl();
                                }
                            }

                            if (VehicleDynamics.Velocity.LateralAbs > 0.5)
                                Odometer += VehicleDynamics.Velocity.LateralAbs * dT;
                        }
                        NavgtnDisplays.Update();
                        StatusDisplays.Update();
                        bool busy = !ConnectorSubrtn.IsConnected && (TCS.Busy || Autopilot.Enabled || CruiseControl.Enabled || UserInput.Manning || VehicleDynamics.Velocity.Scalar > 0.5);
                        if (Busy && !busy)
                        {
                            Parent.Runtime.UpdateFrequency = UpdateFrequency.Update10;
                            Profiler.SkipCycles = 2;
                        }
                        else if (!Busy && busy)
                        {
                            Parent.Runtime.UpdateFrequency = UpdateFrequency.Update1;
                            Profiler.SkipCycles = 14;
                        }
                        Busy = busy;
                    }
                    catch (Exception ex) // *Pokemon joke here*
                    {
                        Booted = false;
                        Parent.Echo("Exeption: " + ex.ToString());
                        VehicleDynamics.Reset();
                        UserInput.Reset();
                        ReleaseAllOverrides();
                        Save();
                    }
                    Profiler.Update();
                }
                else // Run boot sequence
                {
                    if (!BootStMachine.MoveNext())
                        BootStMachine = Boot();
                }
            }

            public void ChangeHandbrakeMode()
            {
                ++_HandbrakeMode;
                if (_HandbrakeMode == HandbrakeMode.END_OF_ENUM)
                    _HandbrakeMode = 0;
                AutoBrakes.Mode = _HandbrakeMode;
            }

            public void ToggleAutopilot(bool reset = true)
            {
                Autopilot.Enabled = !Autopilot.Enabled;
                if (Autopilot.Enabled)
                {
                    Autopilot.ResetWaypointIndex(reset);
                    SetAutopilotControl();
                }
                else
                    SetUserControl();
            }

            public void ChangeAutopilotDriveMode()
            {
                ++Autopilot.DriveMode;
                if (Autopilot.DriveMode == AutopilotDriver.EDriveMode.END_OF_ENUM)
                    Autopilot.DriveMode = AutopilotDriver.EDriveMode.OneWay;
            }

            public void InverseAutopilot()
            {
                Autopilot.InverseRoute();
            }

            public void ToggleCruiseControl()
            {
                CruiseControl.Enabled = !CruiseControl.Enabled;
                if (CruiseControl.Enabled)
                    SetCruiseControl();
                else
                    SetUserControl();
            }

            public void ToggleGPSTracker()
            {
                if (!CruiseControl.Enabled && !Autopilot.Enabled)
                    GPSTracker.Enabled = !GPSTracker.Enabled;
            }

            public void ToggleGPSList()
            {
                ShowGPSList = !ShowGPSList;
                if (ShowGPSList)
                    StatusDisplays.Caption = "Driver Assisting System\\\\GPS Track ";
                else
                    StatusDisplays.Caption = "Driver Assisting System\\\\Status ";
            }

            //-------------------------------------------------------------------
            void Load()
            {
                MyIni config = new MyIni();
                if (config.TryParse(Parent.Storage))
                {
                    if (config.ContainsSection("General"))
                    {
                        config.Get("General", "Run").TryGetBoolean(out Run);
                        config.Get("General", "JumpJets").TryGetBoolean(out _UseJumpJets);
                        config.Get("General", "HDC").TryGetBoolean(out _UseHillDescentControl);
                        config.Get("General", "Odometer").TryGetDouble(out Odometer);
                    }
                    if (config.ContainsSection("Autopilot"))
                    {
                        config.Get("Autopilot", "Enabled").TryGetBoolean(out Autopilot.Enabled);
                        config.Get("Autopilot", "WaitTime").TryGetDouble(out _AutopilotWait);
                        config.Get("Autopilot", "Accuracy").TryGetDouble(out _AutopilotAccuracy);
                        config.Get("Autopilot", "WaypointIndex").TryGetInt32(out Autopilot.WaypointIndx);
                        config.Get("Autopilot", "WaypointSelector").TryGetInt32(out Autopilot.WaypointSelector);
                        int unused = (int)Autopilot.DriveMode;
                        int t;
                        config.Get("Autopilot", "DriveMode").TryGetInt32(out t);
                        Autopilot.DriveMode = (AutopilotDriver.EDriveMode)t;
                    }
                }
            }

            IEnumerator<bool> Boot()
            {
                Parent.Runtime.UpdateFrequency = UpdateFrequency.Update10;
                StringQueue bootLog = new StringQueue(15);
                StringBuilder failList = new StringBuilder("\n");
                IMyTextSurface myLCD = (Parent.Me as IMyTextSurfaceProvider).GetSurface(0);
                NavgtnDisplays.DetectBlocks();
                StatusDisplays.DetectBlocks();
                if (NavgtnDisplays.Ready || StatusDisplays.Ready)
                {
                    StatusDisplays.Caption = NavgtnDisplays.Caption = "Driver Assisting System\\\\Booting... ";
                    bootLog.Append("DAS.DisplayScheduler...   ok");
                }
                else
                {
                    bootLog.Append("DAS.DisplayScheduler...   failure");
                    failList.Append("DAS.DisplayScheduler failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                VehicleDynamics.ForceDetectRemoteControl();
                if (!VehicleDynamics.Ready)
                {
                    bootLog.Append("DAS.GridDynamicsWatcher...   failure\n\nCritical failure during boot:\nNo Remote Control Block found.");
                    Parent.Runtime.UpdateFrequency = UpdateFrequency.Update100;
                    BootPrint(bootLog.GetString(), myLCD);
                    yield break;
                }
                IMyRemoteControl rc = VehicleMass.RC = Gravity.RC = Orientation.RC = VehicleDynamics.RC;
                VehicleMass.InitMasses();
                for (int i = 0; i <= Gravity.SkipCycles; ++i)
                    Gravity.Update();
                for (int i = 0; i <= Orientation.SkipCycles; ++i)
                    Orientation.Update(Gravity.Direction, true);
                bootLog.Append("DAS.GridMassWatcher...   ok");
                bootLog.Append("DAS.GravityWatcher...   ok");
                bootLog.Append("DAS.OrientationComputer...   ok");
                bootLog.Append("DAS.GridDynamicsWatcher...   ok");
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                UserInput.DetectFuncBlocks();
                bootLog.Append("DAS.UserInputWatcher...   ok");
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                Cameras.DetectBlocks(rc);
                if (Cameras.Ready)
                {
                    for (int i = 0; i < (Cameras.SkipCycles + 2) * 2; ++i)
                        Cameras.Update();
                    bootLog.Append("DAS.CameraScheduler...   ok");
                }
                else
                {
                    bootLog.Append("DAS.CameraScheduler...   failure");
                    failList.Append("DAS.CameraScheduler failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                TCS.DetectBlocks(rc);
                if (TCS.Ready)
                    bootLog.Append("DAS.TCS...   ok");
                else
                {
                    bootLog.Append("DAS.TCS...   failure");
                    failList.Append("DAS.TCS failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                FDS.DetectBlocks(rc);
                if (FDS.Ready)
                    bootLog.Append("DAS.FDS...   ok");
                else
                {
                    bootLog.Append("DAS.FDS...   failure");
                    failList.Append("DAS.FDS failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                GSA.DetectBlocks(rc);
                RelayTCSDataToGSA();
                if (GSA.Ready)
                {
                    for (int i = 0; i <= GSA.SkipCycles; ++i)
                        GSA.Update();
                    bootLog.Append("DAS.GSA...   ok");
                }
                else
                {
                    bootLog.Append("DAS.GSA...   failure");
                    failList.Append("DAS.GSA failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                SDO.DetectBlocks(rc);
                if (SDO.Ready)
                    bootLog.Append("DAS.SDO...   ok");
                else
                {
                    bootLog.Append("DAS.SDO...   failure");
                    failList.Append("DAS.SDO failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                Lights.DetectBlocks(rc);
                if (Lights.Ready)
                    bootLog.Append("DAS.LightsManager...   ok");
                else
                {
                    bootLog.Append("DAS.LightsManager...   failure");
                    failList.Append("DAS.LightsManager failure\n");
                }
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                AutoBrakes.RC = CruiseControl.RC = GPSTracker.RC = rc;
                bootLog.Append("DAS.AutoHandbrakesSubroutine...   ok");
                ArtMass.DetectBlocks();
                Autopilot.DetectBlocks(rc);
                if (ArtMass.Ready)
                    bootLog.Append("DAS.ArtificialMassSubroutine...   ok");
                else
                    bootLog.Append("DAS.ArtificialMassSubroutine...   failure");
                bootLog.Append("DAS.AutopilotDriver...   ok");
                bootLog.Append("DAS.CruiseControlSubroutine...   ok");
                bootLog.Append("DAS.GPSTrackerSubroutine...   ok");
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                ConnectorSubrtn.DetectBlocks(rc);
                if (ConnectorSubrtn.Ready)
                    bootLog.Append("DAS.ConnectorSubroutine...   ok");
                else
                    bootLog.Append("DAS.ConnectorSubroutine...   failure");
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                BackupBeeper.DetectBlocks();
                if (BackupBeeper.Ready)
                    bootLog.Append("DAS.BackupBeeperSubroutine...   ok");
                else
                    bootLog.Append("DAS.BackupBeeperSubroutine...   failure");
                BootPrint(bootLog.GetString(), myLCD);
                yield return true;
                NavgtnDisplays.Caption = "Driver Assisting System\\\\Navigation ";
                StatusDisplays.Caption = "Driver Assisting System\\\\Status ";
                Profiler.Memo = failList.ToString();
                Busy = false;
                Booted = true;
            }

            readonly string textDASshutdown = "\n\n\n\n                         DAS shutdown";
            readonly string textDASstandby = "\n\n\n\n                         DAS standby";
            StringBuilder ComposeNavigationData(int i = 0)
            {
                StrN.Clear();
                if (!Run)
                    StrN.Append(textDASshutdown);
                else if (ConnectorSubrtn.IsConnected)
                    StrN.Append(textDASstandby);
                else
                {
                    StrN.AppendFormat("\nLocation:\nGPS:{0}:", Parent.Me.CubeGrid.CustomName);
                    StrN.AppendFormat("{0:F2}:", Parent.Me.CubeGrid.WorldVolume.Center.X);
                    StrN.AppendFormat("{0:F2}:", Parent.Me.CubeGrid.WorldVolume.Center.Y);
                    StrN.AppendFormat("{0:F2}:", Parent.Me.CubeGrid.WorldVolume.Center.Z);
                    StrN.AppendFormat(":\n\nLateral velocity:    {0,-1:N2} km/h\nVertical velocity:   {1,-1:N2} m/s\nAcceleration:         {2,-1:N1} m/c^2\nTraveled distance: {3,-1:N1} km",
                    VehicleDynamics.Velocity.Forward * 3.6,
                    VehicleDynamics.Velocity.Up,
                    VehicleDynamics.Acceleration.AvrEMA,
                    Odometer / 1000.0);
                    StrN.AppendFormat("\n\nPitch: {0,-1:F2}°\nYaw:  {1,-1:F2}°\nRoll:   {2,-1:F2}°",
                    MathHelperD.ToDegrees(Orientation.Pitch) - _NaturalPitchDegree,
                    MathHelperD.ToDegrees(Orientation.Yaw),
                    MathHelperD.ToDegrees(Orientation.Roll));
                }
                return StrN;
            }

            [Flags]
            enum WarningFlags : byte { None, GSA, FDS, HDC = 4 }
            WarningFlags Warns;
            StringBuilder ComposeStatusData(int i = 0)
            {
                StrS.Clear();
                if (!Run)
                    StrS.Append(textDASshutdown);
                else if (ConnectorSubrtn.IsConnected)
                    StrS.Append(textDASstandby);
                else if (ShowGPSList)
                    return GPSTracker.GPSTrackList;
                else
                {
                    StrS.Append("                             Systems\n");
                    if (TCS.Ready)
                    {
                        if (TCS.Enabled)
                            StrS.Append("          [ TCS ]");
                        else
                            StrS.Append("            TCS  ");
                    }
                    else
                        StrS.Append("          [         ]");
                    if (GSA.Ready)
                    {
                        if (GSA.Enabled)
                        {
                            if (GSA.IsAllGyrosWorking || i % 2 == 0)
                                StrS.Append("          [ GSA ]");
                            else
                                StrS.Append("          [         ]");
                        }
                        else
                            StrS.Append("            GSA  ");
                    }
                    else
                        StrS.Append("          [         ]");
                    if (FDS.Ready)
                    {
                        if (FDS.Enabled)
                        {
                            if (FDS.EnoughThrust || i % 2 == 0)
                                StrS.Append("          [ FDS ]");
                            else
                                StrS.Append("          [         ]");
                        }
                        else
                            StrS.Append("            FDS  ");
                    }
                    else
                        StrS.Append("          [         ]");
                    StrS.Append("\n");
                    if (SDO.Ready)
                    {
                        if (SDO.Enabled)
                            StrS.Append("          [ SDO ]");
                        else
                            StrS.Append("            SDO  ");
                    }
                    else
                        StrS.Append("          [         ]");
                    if (Lights.Ready)
                    {
                        if (Lights.Enabled)
                            StrS.Append("          [LGHT]");
                        else
                            StrS.Append("           LGHT ");
                    }
                    else
                        StrS.Append("          [         ]");
                    StrS.Append("\n");
                    if (Warns == WarningFlags.None)
                    {
                        if (GSA.Ready && GSA.Enabled && !GSA.IsAllGyrosWorking)
                            Warns |= WarningFlags.GSA;
                        if (FDS.Ready && FDS.Enabled && !FDS.EnoughThrust)
                            Warns |= WarningFlags.FDS;
                        if (TCS.UseHillDescentControl && !_UseHillDescentControl)
                            Warns |= WarningFlags.HDC;
                    }
                    if (Warns.HasFlag(WarningFlags.GSA))
                    {
                        StrS.Append("   GSA: Some gyros disabled or damaged.\n");
                        if (i % 4 == 0)
                            Warns ^= WarningFlags.GSA;
                    }
                    else if (Warns.HasFlag(WarningFlags.FDS))
                    {
                        StrS.Append("   FDS: Not enough thrust.\n");
                        if (i % 4 == 0)
                            Warns ^= WarningFlags.FDS;
                    }
                    else if (Warns.HasFlag(WarningFlags.HDC))
                    {
                        StrS.Append("   DAS: HDC activated by autopilot.\n");
                        if (i % 4 == 0)
                            Warns ^= WarningFlags.HDC;
                    }
                    else
                        StrS.Append("\n");
                    switch (_HandbrakeMode)
                    {
                        case HandbrakeMode.Semi:
                            StrS.Append("          [SEMI]");
                            break;
                        case HandbrakeMode.Auto:
                            StrS.Append("          [AUTO]");
                            break;
                        case HandbrakeMode.Off:
                            StrS.Append("          [         ]");
                            break;
                    }
                    if (_UseJumpJets)
                        StrS.Append("          [JUMP]");
                    else
                        StrS.Append("          [         ]");
                    if (TCS.UseHillDescentControl && (_UseHillDescentControl || i % 2 == 0))
                        StrS.Append("          [ HDC ]");
                    else
                        StrS.Append("          [         ]");
                    StrS.Append("\n");
                    if (CruiseControl.Enabled)
                        StrS.Append("          [  CC  ]");
                    else
                        StrS.Append("          [         ]");
                    if (GPSTracker.Enabled && i % 2 == 0)
                        StrS.Append("          [¤REC]");
                    else
                        StrS.Append("          [         ]");
                    StrS.Append("\n_________________________________________________________\n");
                    StrS.AppendFormat("Autopilot:              {0}\n", Autopilot.Enabled ? "ACTIVE" : "OFF");
                    StrS.AppendFormat("Accuracy:              {0}\n", _AutopilotAccuracy);
                    StrS.Append("Wait time:              ");
                    if (Autopilot.Time >= 1)
                        StrS.AppendFormat("{0}/", Math.Floor(Autopilot.Time));
                    StrS.AppendFormat("{0} sec\n", _AutopilotWait);
                    switch (Autopilot.DriveMode)
                    {
                        case AutopilotDriver.EDriveMode.OneWay:
                            StrS.Append("Drive Mode:           ONE WAY\n");
                            break;
                        case AutopilotDriver.EDriveMode.Patrol:
                            StrS.Append("Drive Mode:           PATROL\n");
                            break;
                        case AutopilotDriver.EDriveMode.Cicrle:
                            StrS.Append("Drive Mode:           CIRCLE\n");
                            break;
                    }
                    StrS.Append("Route:\n");
                    if (Autopilot.Enabled)
                    {
                        StrS.AppendFormat("{0} {1} {2}\n",
                        Autopilot.FirstWaypoint.Name,
                        i % 3 == 0 ? ">-->" : i % 3 == 1 ? "->--" : "-->-",
                        Autopilot.LastWaypoint.Name);
                    }
                    else
                        StrS.Append("N/A\n");
                    StrS.Append("Current Waypoint:\n");
                    StrS.AppendFormat("{0}\n", Autopilot.Enabled ? Autopilot.CurrentWaypoint.ToString() : "N/A");
                }
                return StrS;
            }

            void BootPrint(StringBuilder str, IMyTextSurface pbLCD)
            {
                NavgtnDisplays.ForcePrint(str);
                StatusDisplays.ForcePrint(str);
                Parent.Echo(str.ToString());
                pbLCD?.WriteText(str);
            }

            void ReleaseAllOverrides()
            {
                try
                {
                    TCS.ReleaseControl();
                    FDS.ReleaseControl();
                    GSA.ReleaseControl();
                    SDO.ReleaseControl();
                    Lights.ReleaseControl();
                }
                finally { }
            }

            void SetAutopilotControl()
            {
                CruiseControl.Enabled = false;
                GPSTracker.Enabled = false;
                TCS.SetInputSource(Autopilot);
                TCS.ForceFullOverride = true;
                TCS.UseHillDescentControl = true;
                TCS.ForwardSpeedLimitKPH = TCS.BackwardSpeedLimitKPH = _AutopilotSpeedLimitKMP;
                GSA.SetInputSource(Autopilot);
                SDO.SetInputSource(Autopilot);
                AutoBrakes.SetInputSource(Autopilot);
                Lights.SetInputSource(Autopilot);
            }

            void SetCruiseControl()
            {
                Autopilot.Enabled = false;
                GPSTracker.Enabled = false;
                TCS.SetInputSource(CruiseControl);
                TCS.ForceFullOverride = true;
                TCS.UseHillDescentControl = _UseHillDescentControl;
                GSA.SetInputSource(CruiseControl);
                SDO.SetInputSource(CruiseControl);
                AutoBrakes.SetInputSource(CruiseControl);
                Lights.SetInputSource(CruiseControl);
            }

            void SetUserControl()
            {
                TCS.SetInputSource(UserInput);
                TCS.ForceFullOverride = false;
                TCS.UseHillDescentControl = _UseHillDescentControl;
                TCS.ForwardSpeedLimitKPH = _FrwrdSpeedLimitKPH;
                TCS.BackwardSpeedLimitKPH = _BckwrdSpeedLimitKPH;
                TCS.ReleaseControl();
                GSA.SetInputSource(UserInput);
                SDO.SetInputSource(UserInput);
                AutoBrakes.SetInputSource(UserInput);
                Lights.SetInputSource(UserInput);
            }

            void RelayTCSDataToGSA()
            {
                GSA.CoMToWheelsBaseAngle = TCS.CoMToWheelsBaseAngle;
                GSA.VehicleTurnRadiusLeft = TCS.VehicleTurnRadiusLeft;
                GSA.VehicleTurnRadiusRight = TCS.VehicleTurnRadiusRight;
            }

        }

        //-------------------------------------------------------------------

        readonly DriverAssistingSystem DAS;

        public Program()
        {
            DAS = new DriverAssistingSystem(this);
        }

        public void Save()
        {
            if (DAS.Run)
                DAS.Save();
        }

        public void Main(string args, UpdateType updateSource)
        {
            if (updateSource.HasFlag(UpdateType.Terminal) || updateSource.HasFlag(UpdateType.Trigger))
            {
                switch (args.ToUpperInvariant())
                {
                    case "START":
                        DAS.Start();
                        break;
                    case "STOP":
                        DAS.Stop();
                        break;
                    case "STARTSTOP":
                        if (DAS.Run)
                        {
                            DAS.Stop();
                        }
                        else
                        {
                            DAS.Start();
                        }
                        break;
                    case "LOW_MODE":
                        DAS.SuspensionValueChangeRate = 60;
                        DAS.SuspensionHeightOffset = 0;
                        break;
                    case "HIGH_MODE":
                        DAS.SuspensionValueChangeRate = 15;
                        DAS.SuspensionHeightOffset = -1.5;
                        break;
                    case "RESET_ODOMETER":
                        DAS.Odometer = 0;
                        DAS.Save();
                        break;
                    case "CYCLE_HANDBRAKES":
                        DAS.ChangeHandbrakeMode();
                        break;
                    case "TOGGLE_JUMPJETS":
                        DAS.UseJumpJets = !DAS.UseJumpJets;
                        break;
                    case "TOGGLE_GSA":
                        DAS.UseGSA = !DAS.UseGSA;
                        break;
                    case "TOGGLE_HDC":
                        DAS.UseHillDescentControl = !DAS.UseHillDescentControl;
                        break;
                    case "TOGGLE_AUTOPILOT":
                        DAS.ToggleAutopilot();
                        break;
                    case "RESUME_AUTOPILOT":
                        DAS.ToggleAutopilot(false);
                        break;
                    case "CYCLE_AUTOPILOT_ACC":
                        DAS.AutopilotAccuracy = DAS.AutopilotAccuracy % 2.5 + 0.25;
                        break;
                    case "CYCLE_AUTOPILOT_WAIT":
                        DAS.AutopilotWait = (DAS.AutopilotWait + 5) % 65;
                        break;
                    case "CYCLE_AUTOPILOT_MODE":
                        DAS.ChangeAutopilotDriveMode();
                        break;
                    case "INVERSE_AUTOPILOT":
                        DAS.InverseAutopilot();
                        break;
                    case "TOGGLE_CRUISECONTROL":
                        DAS.ToggleCruiseControl();
                        break;
                    case "TOGGLE_GPSTRACKER":
                        DAS.ToggleGPSTracker();
                        break;
                    case "TOGGLE_GPSLIST":
                        DAS.ToggleGPSList();
                        break;
                    default:
                        break;
                }
            }
            else
            {
                DAS.Update(updateSource);
            }
        }
    }
}
