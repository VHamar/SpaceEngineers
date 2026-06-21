# Inventory Control

A Space Engineers mod that adds an **Inventory Control Block** — a programmable block variant used as the foundation for automated inventory management.

## Structure

```
Modules/InventoryControl/
├── modinfo.sbmi                          # Mod metadata (Steam Workshop info)
├── README.md
└── Data/
    └── CubeBlocks/
        └── CubeBlocks_InventoryControl.sbc   # Block definitions
```

## Blocks

### Inventory Control Block
| Property      | Large Grid           | Small Grid           |
|---------------|----------------------|----------------------|
| Size          | 1 × 2 × 1            | 1 × 1 × 1            |
| PCU           | 5                    | 5                    |
| Build Time    | 30 s                 | 15 s                 |
| Model         | Vanilla PB (Large)   | Vanilla PB (Small)   |

**Recipe (Large)**
| Component         | Count |
|-------------------|-------|
| Steel Plate       | 21    |
| Construction Comp | 7     |
| Large Steel Tube  | 2     |
| Motor             | 1     |
| Display           | 1     |
| Bulletproof Glass | 1     |
| Computer *(critical)* | 2 |

## Deployment

Copy the `Modules/InventoryControl` folder to your Space Engineers local mods directory:

```
%AppData%\SpaceEngineers\Mods\InventoryControl\
```

Then enable the mod in the game's world settings under **Mods**.

## Development Notes

- Block SubtypeIds: `InventoryControlBlock_Large` / `InventoryControlBlock_Small`
- TypeId: `MyObjectBuilder_MyProgrammableBlock`
- Both sizes share the `BlockPairName` `InventoryControlBlock` so they appear as a single entry in the toolbar
