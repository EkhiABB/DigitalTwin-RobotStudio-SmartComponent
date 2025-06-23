using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Configuration;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.IOSystemDomain;
using ABB.Robotics.Math;
using ABB.Robotics.RobotStudio;
using ABB.Robotics.RobotStudio.Controllers;
using ABB.Robotics.RobotStudio.Stations;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.Remoting.Contexts;
using System.Text;

namespace Digitaltwin
{
    /// <summary>
    /// Code-behind class for the Digitaltwin Smart Component.
    /// </summary>
    /// <remarks>
    /// The code-behind class should be seen as a service provider used by the 
    /// Smart Component runtime. Only one instance of the code-behind class
    /// is created, regardless of how many instances there are of the associated
    /// Smart Component.
    /// Therefore, the code-behind class should not store any state information.
    /// Instead, use the SmartComponent.StateCache collection.
    /// </remarks>
    public class CodeBehind : SmartComponentCodeBehind
    {
        private Controller controller;

        public override void OnSimulationStart(SmartComponent component)
        {
            component.IOSignals["AddIoSignal"].ReadOnly = true;
            component.IOSignals["RemoveIoSignal"].ReadOnly = true;
            component.IOSignals["RemoveIoSignalAll"].ReadOnly = true;
        }

        public override void OnSimulationStop(SmartComponent component)
        {
            component.IOSignals["AddIoSignal"].ReadOnly = false;
            component.IOSignals["RemoveIoSignal"].ReadOnly = false;
            component.IOSignals["RemoveIoSignalAll"].ReadOnly = false;
        }

        /// <summary>
        /// Called when the value of a dynamic property value has changed.
        /// </summary>
        /// <param name="component"> Component that owns the changed property. </param>
        /// <param name="changedProperty"> Changed property. </param>
        /// <param name="oldValue"> Previous value of the changed property. </param>
        public override void OnPropertyValueChanged(SmartComponent component, DynamicProperty changedProperty, Object oldValue)
        {
        }

        /// <summary>
        /// Called when the value of an I/O signal value has changed.
        /// </summary>
        /// <param name="component"> Component that owns the changed signal. </param>
        /// <param name="changedSignal"> Changed signal. </param>
        public override void OnIOSignalValueChanged(SmartComponent component, IOSignal changedSignal)
        {
            if (changedSignal.GroupName == "Signal Monitoring")
            {
                return;
            }

            if ((int)changedSignal.Value == 1)
            {
                switch (changedSignal.Name)
                {
                    case "Connect":
                        TryConnectRealController(component);
                        break;
                    case "Disconnect":
                        Disconnect(component);
                        break;
                    case "AddIoSignal":
                        TryAddIoSignal(component);
                        break;
                    case "RemoveIoSignal":
                        TryRemoveIoSignal(component);
                        break;
                    case "RemoveIoSignalAll":
                        TryRemoveIoSignalAll(component);
                        break;
                    default:
                        break;
                }
            }
        }

        /// <summary>
        /// Called during simulation.
        /// </summary>
        /// <param name="component"> Simulated component. </param>
        /// <param name="simulationTime"> Time (in ms) for the current simulation step. </param>
        /// <param name="previousTime"> Time (in ms) for the previous simulation step. </param>
        /// <remarks>
        /// For this method to be called, the component must be marked with
        /// simulate="true" in the xml file.
        /// </remarks>
        public override void OnSimulationStep(SmartComponent component, double simulationTime, double previousTime)
        {
            if (controller != null && controller.Connected)
            {
                MonitorMechanism(component);
                MonitorSignals(component);
            }
            else
            {
                component.IOSignals["Connected"].Value = 0;
            }
        }

        private void TryConnectRealController(SmartComponent component)
        {
            component.IOSignals["Connected"].Value = 0;
            NetworkScanner scanner = new NetworkScanner();
            scanner.Scan();
            var ipadress = (string)component.Properties["IpAdress"].Value;
            var info = scanner.Controllers.FirstOrDefault(e => e.IPAddress.ToString() == ipadress);
            if (info != null)
            {
                controller = Controller.Connect(info,
                            ConnectionType.Standalone
                            );
                if (controller.Connected)
                {
                    component.IOSignals["Connected"].Value = 1;
                }
                else
                {
                    controller = null;
                }
            }
            else
            {
                controller = null;
            }

        }

        private void Disconnect(SmartComponent component)
        {
            if (controller != null)
            {
                controller.Dispose();
                controller = null;
            }
            component.IOSignals["Connected"].Value = 0;
        }

        private void TryAddIoSignal(SmartComponent component)
        {
            var signalName = (string)component.Properties["IoSignalName"].Value;
            if (signalName != "")
            {
                if (controller != null && controller.Connected)
                {
                    var ctrlsignal = controller.IOSystem.GetSignal(signalName);
                    if (ctrlsignal != null)
                    {
                        var scsignal = new IOSignal(signalName, GetIOSignalType(ctrlsignal));
                        scsignal.GroupName = "Signal Monitoring";
                        scsignal.ReadOnly = true;
                        component.DisconnectFromLibrary();
                        component.IOSignals.Add(scsignal);
                    }
                }
            }
        }

        private void TryRemoveIoSignal(SmartComponent component)
        {
            var signalName = (string)component.Properties["IoSignalName"].Value;
            if (signalName != "")
            {
                var scsignal = component.IOSignals.FirstOrDefault(s => s.Name == signalName && s.GroupName == "Signal Monitoring");
                if (scsignal != null)
                {
                    component.DisconnectFromLibrary();
                    component.IOSignals.Remove(signalName);
                }
            }
        }

        private void TryRemoveIoSignalAll(SmartComponent component)
        {
            List<string> signalNames =
                component.IOSignals.Where(s => s.GroupName == "Signal Monitoring")
                .Select(s => s.Name)
                .ToList();

            component.DisconnectFromLibrary();
            foreach (var name in signalNames)
            {
                component.IOSignals.Remove(name);
            }

        }

        private void MonitorMechanism(SmartComponent component)
        {
            var mech = (Mechanism)component.Properties["Mechanism"].Value;
            if (mech != null)
            {
                var joints = controller.MotionSystem.MechanicalUnits.First().GetPosition();
                mech.SetJointValues(new double[]
                {
                        joints.RobAx.Rax_1*Math.PI/180.0,
                        joints.RobAx.Rax_2*Math.PI/180.0,
                        joints.RobAx.Rax_3*Math.PI/180.0,
                        joints.RobAx.Rax_4*Math.PI/180.0,
                        joints.RobAx.Rax_5*Math.PI/180.0,
                        joints.RobAx.Rax_6*Math.PI/180.0
                }, false);
            }
        }

        private void MonitorSignals(SmartComponent component)
        {
            foreach (var scsignal in component.IOSignals.Where(s => s.GroupName == "Signal Monitoring"))
            {
                var ctrlsignal = controller.IOSystem.GetSignal(scsignal.Name);
                if (ctrlsignal != null)
                {
                    scsignal.TrySetValue(ctrlsignal.Value);
                }
            }
        }

        private IOSignalType GetIOSignalType(Signal controllerSignal)
        {
            switch (controllerSignal.Type)
            {
                case SignalType.DigitalInput:
                    return IOSignalType.DigitalOutput;
                case SignalType.DigitalOutput:
                    return IOSignalType.DigitalOutput;
                case SignalType.GroupInput:
                    return IOSignalType.DigitalGroupOutput;
                case SignalType.GroupOutput:
                    return IOSignalType.DigitalGroupOutput;
                case SignalType.AnalogInput:
                    return IOSignalType.AnalogOutput;
                case SignalType.AnalogOutput:
                    return IOSignalType.AnalogOutput;
                default:
                    return IOSignalType.DigitalOutput;
            }
        }

    }
}
