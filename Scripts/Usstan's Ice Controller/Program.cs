﻿using Sandbox.Game.EntityComponents;
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
        #region mdk preserve
        // ---=== Usstan's Ice Controller ===---
        // This script was created for use on worlds with water (WaterMod) where pumping ice from the water
        // can quickly result in all cargo containers on the base being full of ice. I originally used 
        // Coxy's Ice Pump, but wanted a little more control and configuration

        // Limit for ice in containers.
        private readonly long maxIceAmount = 10000;

        // Put all Collectors you want to control in a group with this name
        // Also add any LCDs that you want to display info on in this group
        // Yes, you can change the group name
        private readonly string groupName = "Ice Control";

        // Limit to current grid, setting this to false will make it count ice in all connected grids.
        private readonly bool sameGridOnly = true;

        private readonly bool showHeading = true;
        // Don't modify below this line
        #endregion

        private readonly IMyTextSurface programBlockScreen;

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            programBlockScreen = Me.GetSurface(0);
        }

        public void Main(string argument, UpdateType updateSource)
        {
            var pumpGroup = GridTerminalSystem.GetBlockGroupWithName(groupName);
            if (pumpGroup == null)
            {
                Echo("Can't find group: " + groupName);
                return;
            }


            var icePumps = new List<IMyCollector>();
            pumpGroup.GetBlocksOfType(icePumps);
            Echo("Pumps found: " + icePumps.Count);

            var iceLcds = new List<IMyTextPanel>();
            pumpGroup.GetBlocksOfType(iceLcds);
            Echo("Lcds found: " + iceLcds.Count);

            var iceAmount = CountIce();

            foreach (var pump in icePumps)
            {
                pump.Enabled = iceAmount < maxIceAmount; 
            }

            UpdateLcds(iceLcds, programBlockScreen, CreateScreenOutput(icePumps, iceAmount));
        }

        private int CountIce()
        {
            var iceAmount = 0;

            var cargoContainers = new List<IMyCargoContainer>();
            if (sameGridOnly)
            {
                GridTerminalSystem.GetBlocksOfType(cargoContainers, container => container.IsSameConstructAs(Me));
            }
            else
            {
                GridTerminalSystem.GetBlocksOfType(cargoContainers);
            }

            Echo("Found Ice in:");
            foreach (var cargoContainer in cargoContainers)
            {
                var items = new List<MyInventoryItem>();
                cargoContainer.GetInventory().GetItems(items, item => item.ToString().Contains("MyObjectBuilder_Ore/Ice"));
                foreach (var ice in items)
                {
                    Echo(cargoContainer.CustomName + ": " + ice.Amount);
                    iceAmount += (int)ice.Amount;
                }
            }
            Echo("");
            return iceAmount;
        }

        private string CreateScreenOutput(List<IMyCollector> icePumps, int iceAmount)
        {
            var output = "";
            if (showHeading)
            {
                output = "---=== Usstan's Ice Controller ===---\n\n";
            }

            output += "Ice in storage: " + iceAmount + "/" + maxIceAmount + "\n" +
                "\n" +
                groupName + ":\n";

            foreach (var pump in icePumps)
            {
                string status;
                if (pump.Enabled)
                {
                    status = "Running";
                }
                else
                {
                    status = "Stopped";
                }
                output += pump.CustomName.PadRight(35).Substring(0, 30) + " " + status + "\n";
            }
            return output;
        }

        private void UpdateLcds(List<IMyTextPanel> panels, IMyTextSurface pbScreen, string output)
        {
            WriteScreen(pbScreen, output);
            foreach (var screen in panels)
            {
                WriteScreen(screen, output);
            }
            Echo(output);
        }

        private void WriteScreen(IMyTextSurface surface, string output)
        {
            surface.ContentType = ContentType.TEXT_AND_IMAGE;
            surface.FontSize = 1.0f;
            surface.WriteText(output);
        }
    }
}
