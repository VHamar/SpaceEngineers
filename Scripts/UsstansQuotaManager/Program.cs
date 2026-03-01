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
    public partial class Program : MyGridProgram
    {
        #region mdk preserve

        // Keyword to identify managed blocks, if you want to use something else than [Managed], change it here.
        string UQM_Keyword = "[Managed]";

        #endregion


        public Program()
        {
            // Run every 100 ticks (~1.6 seconds)
            Runtime.UpdateFrequency = UpdateFrequency.Update100; 
        }

        public void Main(string argument, UpdateType updateSource)
        {
            

            // Clear the screen
            Echo("");
            
            // Build inventory of all items in containers
            Dictionary<string, double> totalInventory = BuildInventory();
            
            // Get all managed blocks on all connected grids
            List<IMyTerminalBlock> allBlocks = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(allBlocks, block => 
                block.IsSameConstructAs(Me) && 
                block.CustomName.Contains(UQM_Keyword)
            );
            
            // Process each managed block and collect status
            List<BlockStatus> statusList = new List<BlockStatus>();
            foreach (var block in allBlocks)
            {
                BlockStatus status = ProcessManagedBlock(block, totalInventory);
                statusList.Add(status);
            }
            
            // Draw to the programmable block's LCD surface
            IMyTextSurface surface = Me.GetSurface(0);
            bool isAutomaton = Me.BlockDefinition.SubtypeId.Contains("Reskin");
            long surfaceId = Me.EntityId; // Use PB EntityId for its surface
            DrawStatusDisplay(surface, surfaceId, statusList, allBlocks.Count, totalInventory.Count, isAutomaton, SCROLL_SPEED, 0.55f);
            
            // Draw to all [UQM-Status] LCD monitors
            List<IMyTextPanel> urmDisplays = new List<IMyTextPanel>();
            GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(urmDisplays, panel => 
                panel.IsSameConstructAs(Me) && 
                panel.CustomName.Contains("[UQM-Status]")
            );
            
            foreach (var panel in urmDisplays)
            {
                float panelScrollSpeed, panelFontSize;
                ParsePanelConfig(panel, out panelScrollSpeed, out panelFontSize);
                DrawStatusDisplay(panel, panel.EntityId, statusList, allBlocks.Count, totalInventory.Count, false, panelScrollSpeed, panelFontSize);
            }
            
            // Write inventory list to programmable block's Custom Data
            UpdateInventoryListInCustomData(totalInventory);
            
            // Also echo summary to status window
            Echo(string.Format("Managed Blocks: {0}", allBlocks.Count));
            Echo(string.Format("UQM-Status Displays: {0}", urmDisplays.Count));
            Echo(string.Format("Inventory Types: {0}", totalInventory.Count));
        }

        private Dictionary<string, double> BuildInventory()
        {
            Dictionary<string, double> inventory = new Dictionary<string, double>();
            
            // Get all blocks with inventory on the current grid
            List<IMyTerminalBlock> containersWithInventory = new List<IMyTerminalBlock>();
            GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(containersWithInventory, block => 
                block.CubeGrid == Me.CubeGrid && 
                block.HasInventory
            );
            
            // Iterate through all containers
            foreach (var container in containersWithInventory)
            {
                // Check all inventory slots (some blocks have multiple)
                for (int i = 0; i < container.InventoryCount; i++)
                {
                    IMyInventory containerInventory = container.GetInventory(i);
                    List<MyInventoryItem> items = new List<MyInventoryItem>();
                    containerInventory.GetItems(items);
                    
                    // Add each item to the total inventory
                    foreach (var item in items)
                    {
                        string itemKey = string.Format("{0}:{1}", item.Type.TypeId, item.Type.SubtypeId);
                        double amount = (double)item.Amount;
                        
                        if (inventory.ContainsKey(itemKey))
                        {
                            inventory[itemKey] += amount;
                        }
                        else
                        {
                            inventory[itemKey] = amount;
                        }
                    }
                }
            }
            
            return inventory;
        }

        private void ParsePanelConfig(IMyTextPanel panel, out float scrollSpeed, out float fontSize)
        {
            // Default values
            scrollSpeed = SCROLL_SPEED;
            fontSize = 0.55f;
            
            string customData = panel.CustomData;
            
            // Check if config section exists
            if (!customData.Contains("[UQM-Config]"))
            {
                // Add config section
                string configSection = "[UQM-Config]\n" +
                                      "// Configuration for this display\n" +
                                      "// ScrollSpeed: Speed of scrolling (default 10)\n" +
                                      "// FontSize: Size of text (default 0.55)\n" +
                                      "ScrollSpeed=10\n" +
                                      "FontSize=0.55\n" +
                                      "[/UQM-Config]\n";
                
                if (string.IsNullOrWhiteSpace(customData))
                {
                    panel.CustomData = configSection;
                }
                else
                {
                    panel.CustomData = customData.TrimEnd() + "\n\n" + configSection;
                }
                return;
            }
            
            // Parse config section
            int startIndex = customData.IndexOf("[UQM-Config]");
            int endIndex = customData.IndexOf("[/UQM-Config]");
            
            if (startIndex >= 0 && endIndex > startIndex)
            {
                string configSection = customData.Substring(startIndex, endIndex - startIndex);
                string[] lines = configSection.Split('\n');
                
                foreach (string line in lines)
                {
                    string trimmedLine = line.Trim();
                    if (trimmedLine.StartsWith("ScrollSpeed="))
                    {
                        float value;
                        if (float.TryParse(trimmedLine.Substring(12), out value))
                        {
                            scrollSpeed = value;
                        }
                    }
                    else if (trimmedLine.StartsWith("FontSize="))
                    {
                        float value;
                        if (float.TryParse(trimmedLine.Substring(9), out value))
                        {
                            fontSize = value;
                        }
                    }
                }
            }
        }

        private void UpdateInventoryListInCustomData(Dictionary<string, double> inventory)
        {
            string currentData = Me.CustomData;
            
            // Remove old [UQM] section if it exists
            int startIndex = currentData.IndexOf("[UQM]");
            int endIndex = currentData.IndexOf("[/UQM]");
            
            string beforeSection = "";
            string afterSection = "";
            
            if (startIndex >= 0 && endIndex >= 0)
            {
                beforeSection = currentData.Substring(0, startIndex);
                afterSection = currentData.Substring(endIndex + 6); // +6 for "[/UQM]"
            }
            else
            {
                beforeSection = currentData;
            }
            
            // Build the new [UQM] section
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("[UQM]");
            sb.AppendLine("// Inventory Item Types");
            sb.AppendLine("// Auto-generated list - do not edit");
            sb.AppendLine();
            
            // Sort the keys alphabetically for easier reading
            List<string> sortedKeys = new List<string>(inventory.Keys);
            sortedKeys.Sort();
            
            string lastResourceType = string.Empty;
            foreach (string itemKey in sortedKeys)
            {
                string resourceType = itemKey.Split(':')[0];
                if (resourceType != lastResourceType && lastResourceType != string.Empty)
                {
                    sb.AppendLine();
                }
                lastResourceType = resourceType;
                sb.AppendLine(itemKey);
            }
            
            sb.AppendLine("[/UQM]");
            
            // Combine sections
            Me.CustomData = (beforeSection.TrimEnd() + "\n\n" + sb.ToString() + afterSection).Trim();
        }

        private void DrawStatusDisplay(IMyTextSurface surface, long surfaceId, List<BlockStatus> statusList, int managedCount, int inventoryCount, bool isAutomaton, float scrollSpeed, float fontSize)
        {
            surface.ContentType = ContentType.SCRIPT;
            surface.Script = "";
            surface.ScriptBackgroundColor = Color.Black;
            surface.ScriptForegroundColor = Color.White;
            
            Vector2 screenSize = surface.SurfaceSize;
            Vector2 screenCenter = screenSize / 2f;
            
            // X offset for Automaton PB
            float xOffset = isAutomaton ? 50f : 0f;
            
            // Calculate line height: font height + small static spacer
            const float rowSpacer = 3f; // Narrow static spacing between rows
            float lineHeight = fontSize * 55f + rowSpacer;
            
            using (var frame = surface.DrawFrame())
            {
                // Background
                var background = new MySprite(SpriteType.TEXTURE, "SquareSimple", 
                    screenCenter, screenSize, Color.Black);
                frame.Add(background);
                
                // Adjust starting position: 0 for LCD panels, 50 for Automaton PB, 100 for normal PB
                float yPos;
                if (surface is IMyTextPanel)
                {
                    yPos = 0f;
                }
                else if (isAutomaton)
                {
                    yPos = 50f;
                }
                else
                {
                    yPos = 100f;
                }
                
                float startYPos = yPos;
                
                // Header (not scrolled)
                var headerBg = new MySprite(SpriteType.TEXTURE, "SquareSimple", 
                    new Vector2(screenCenter.X + xOffset, yPos + 20), 
                    new Vector2(screenSize.X - 20, 50), 
                    new Color(20, 40, 60));
                frame.Add(headerBg);
                
                var headerText = new MySprite(SpriteType.TEXT, "Usstan's Quota Manager", 
                    new Vector2(screenCenter.X + xOffset, yPos), 
                    null, Color.White, "White", TextAlignment.CENTER, 1.0f);
                frame.Add(headerText);
                
                yPos += 55f;
                
                // Summary info (not scrolled)
                var summaryText = new MySprite(SpriteType.TEXT, 
                    string.Format("Managed: {0}  |  Inventory: {1}", managedCount, inventoryCount),
                    new Vector2(screenCenter.X + xOffset, yPos), 
                    null, new Color(200, 200, 200), "White", TextAlignment.CENTER, 0.6f);
                frame.Add(summaryText);
                
                yPos += 35f;
                
                // Save position where list starts
                float listStartYPos = yPos;
                
                // Get or create scroll state for this display
                if (!displayScrollStates.ContainsKey(surfaceId))
                {
                    displayScrollStates[surfaceId] = new ScrollState();
                }
                ScrollState scrollState = displayScrollStates[surfaceId];
                
                // Apply scroll offset only to the list
                yPos -= scrollState.Offset;
                
                // Status entries (scrolled)
                foreach (var status in statusList)
                {
                    // Skip drawing if item would overlap the header/summary area
                    if (yPos < listStartYPos)
                    {
                        yPos += lineHeight;
                        continue;
                    }
                    
                    // Determine color based on status
                    Color statusColor;
                    string statusIcon;
                    
                    if (status.Status == "RUNNING")
                    {
                        statusColor = new Color(50, 200, 50); // Green
                        statusIcon = "IconEnergy";
                    }
                    else if (status.Status == "STOPPED")
                    {
                        statusColor = new Color(200, 50, 50); // Red
                        statusIcon = "Cross";
                    }
                    else if (status.Status == "CONFIG")
                    {
                        statusColor = new Color(200, 200, 50); // Yellow
                        statusIcon = "Circle";
                    }
                    else // ERROR
                    {
                        statusColor = new Color(200, 100, 0); // Orange
                        statusIcon = "IconWarning";
                    }
                    
                    // Status bar background
                    var statusBg = new MySprite(SpriteType.TEXTURE, "SquareSimple",
                        new Vector2(screenCenter.X + xOffset, yPos + 12),
                        new Vector2(screenSize.X - 40, 28),
                        new Color(statusColor.R / 4, statusColor.G / 4, statusColor.B / 4));
                    frame.Add(statusBg);
                    
                    // Status indicator (left side)
                    var statusIndicator = new MySprite(SpriteType.TEXTURE, "SquareSimple",
                        new Vector2(30 + xOffset, yPos + 12),
                        new Vector2(8, 28),
                        statusColor);
                    frame.Add(statusIndicator);
                    
                    // Status icon
                    var icon = new MySprite(SpriteType.TEXTURE, statusIcon,
                        new Vector2(60 + xOffset, yPos + 12),
                        new Vector2(20, 20),
                        statusColor);
                    frame.Add(icon);
                    
                    // Block name text (strip UQM_Keyword)
                    string displayName = status.BlockName.Replace(UQM_Keyword, "").Trim();
                    string displayText = displayName;
                    if (status.Status == "ERROR" && status.ErrorMessage != null)
                    {
                        displayText = string.Format("{0} - {1}", displayName, status.ErrorMessage);
                    }
                    
                    var nameText = new MySprite(SpriteType.TEXT, displayText,
                        new Vector2(90 + xOffset, yPos),
                        null, Color.White, "White", TextAlignment.LEFT, fontSize);
                    frame.Add(nameText);
                    
                    // Add quantity/quota column (right-aligned)
                    if (status.Status == "RUNNING" || status.Status == "STOPPED")
                    {
                        string quantityText = string.Format("{0:N0}/{1:N0}", status.CurrentAmount, status.QuotaAmount);
                        var quantitySprite = new MySprite(SpriteType.TEXT, quantityText,
                            new Vector2(screenSize.X - 30 + xOffset, yPos),
                            null, new Color(180, 180, 180), "White", TextAlignment.RIGHT, fontSize * 0.9f);
                        frame.Add(quantitySprite);
                    }
                    
                    yPos += lineHeight;
                }
                
                // Calculate max scroll (if content exceeds screen)
                // Use list start position instead of overall start position
                float contentHeight = yPos + scrollState.Offset - listStartYPos;
                float availableHeight = screenSize.Y - listStartYPos + startYPos;
                float maxScroll = Math.Max(0, contentHeight - availableHeight + 40);
                
                // Update scroll position for this display
                if (maxScroll > 0)
                {
                    scrollState.Offset += scrollSpeed * scrollState.Direction;
                    
                    // Reverse direction at bounds
                    if (scrollState.Offset >= maxScroll)
                    {
                        scrollState.Offset = maxScroll;
                        scrollState.Direction = -1;
                    }
                    else if (scrollState.Offset <= 0)
                    {
                        scrollState.Offset = 0;
                        scrollState.Direction = 1;
                    }
                }
                else
                {
                    scrollState.Offset = 0;
                }
            }
        }

        private BlockStatus ProcessManagedBlock(IMyTerminalBlock block, Dictionary<string, double> inventory)
        {
            BlockStatus status = new BlockStatus();
            status.BlockName = block.CustomName;
            
            string customData = block.CustomData;
            
            // Check if the custom data contains a [Managed] section
            if (!HasManagedSection(customData))
            {
                AddManagedSection(block);
                status.Status = "CONFIG";
                return status;
            }
            
            // Parse the quota from custom data
            string quotaSetting = GetQuotaSetting(customData);
            if (string.IsNullOrEmpty(quotaSetting))
            {
                status.Status = "ERROR";
                status.ErrorMessage = "No Quota";
                return status;
            }
            
            // Check if still using default placeholder quota
            if (quotaSetting == "MyObjectBuilder_Ore:Foo:0")
            {
                status.Status = "CONFIG";
                return status;
            }
            
            // Parse quota: ResourceType:ResourceName:Amount
            string[] quotaParts = quotaSetting.Split(':');
            if (quotaParts.Length != 3)
            {
                status.Status = "ERROR";
                status.ErrorMessage = "Invalid Format";
                return status;
            }
            
            string itemKey = string.Format("{0}:{1}", quotaParts[0], quotaParts[1]);
            string resourceName = quotaParts[1];
            double quotaAmount;
            if (!double.TryParse(quotaParts[2], out quotaAmount))
            {
                status.Status = "ERROR";
                status.ErrorMessage = "Invalid Amount";
                return status;
            }
            
            // Get current inventory amount
            double currentAmount = inventory.ContainsKey(itemKey) ? inventory[itemKey] : 0;
            
            // Store amounts in status
            status.CurrentAmount = currentAmount;
            status.QuotaAmount = quotaAmount;
            
            // Check if block can be enabled/disabled
            IMyFunctionalBlock functionalBlock = block as IMyFunctionalBlock;
            if (functionalBlock == null)
            {
                status.Status = "ERROR";
                status.ErrorMessage = "Not Functional";
                return status;
            }
            
            // Control block based on quota
            if (currentAmount >= quotaAmount)
            {
                if (functionalBlock.Enabled)
                {
                    functionalBlock.Enabled = false;
                }
                status.Status = "STOPPED";
            }
            else
            {
                if (!functionalBlock.Enabled)
                {
                    functionalBlock.Enabled = true;
                }
                status.Status = "RUNNING";
            }
            
            return status;
        }

        private string GetQuotaSetting(string customData)
        {
            // Find the Quota line within the [Quota Managed] section only
            int startIndex = customData.IndexOf("[Quota Managed]");
            int endIndex = customData.IndexOf("[/[Quota Managed]]");
            
            if (startIndex < 0 || endIndex < 0 || endIndex <= startIndex)
            {
                return null;
            }
            
            // Extract only the managed section
            string managedSection = customData.Substring(startIndex, endIndex - startIndex);
            
            string[] lines = managedSection.Split('\n');
            foreach (string line in lines)
            {
                string trimmedLine = line.Trim();
                if (trimmedLine.StartsWith("Quota=") && !trimmedLine.StartsWith("//"))
                {
                    return trimmedLine.Substring(6); // Remove "Quota=" prefix
                }
            }
            return null;
        }

        private bool HasManagedSection(string customData)
        {
            // Check if [Managed] section exists
            return customData.Contains("[Quota Managed]");
        }

        private void AddManagedSection(IMyTerminalBlock block)
        {
            string currentData = block.CustomData;
            string managedSection = "[Quota Managed]\n" +
                                    "// Example configuration\n" +
                                    "// Quota=ResourceType:ResourceName:Amount\n" +
                                    "// Examples:\n" +
                                    "// Quota=MyObjectBuilder_Ore:Ice:5000\n" +
                                    "// Quota=MyObjectBuilder_Ore:Stone:2000\n" +
                                    "// Quota=MyObjectBuilder_PhysicalObject:Algae:100\n" +
                                    "//\n" +
                                    "// You can get a full list of resource types by looking \n" +
                                    "// in the Programmable Blocks Custom Data.\n" +
                                    "\n" +
                                    "Quota=MyObjectBuilder_Ore:Foo:0\n" +
                                    "[/[Quota Managed]]\n";
            
            // If custom data is empty or only whitespace, add the section
            if (string.IsNullOrWhiteSpace(currentData))
            {
                block.CustomData = managedSection;
            }
            else
            {
                // Append to existing custom data with a separator
                block.CustomData = currentData.TrimEnd() + "\n\n" + managedSection;
            }
        }

        // Scroll state - persists between executions (per display)
        Dictionary<long, ScrollState> displayScrollStates = new Dictionary<long, ScrollState>();
        const float SCROLL_SPEED = 10f;
        
        class ScrollState
        {
            public float Offset = 0f;
            public int Direction = 1; // 1 = down, -1 = up
        }

    }

    public class BlockStatus
    {
        public string BlockName;
        public string Status; // RUNNING, STOPPED, CONFIG, ERROR
        public string ErrorMessage;
        public double CurrentAmount;
        public double QuotaAmount;
    }


}
