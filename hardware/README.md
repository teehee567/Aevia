# Hardware

Each board revision keeps its KiCad project, schematic sheets and local libraries together. Open the `.kicad_pro` file in KiCad to work on a board.

| Directory | Contents | Entry point |
| --- | --- | --- |
| [v2_mini_pcb/](v2_mini_pcb/) | V2 Mini electronics and manufacturing files | [KiCad project](v2_mini_pcb/v2_mini.kicad_pro) |
| [v2_mini_case/](v2_mini_case/) | Editable STEP enclosure parts and assembly | [Enclosure assembly](v2_mini_case/enclosure_assembly.step) |
| [v2_pcb/](v2_pcb/) | Full-size V2 electronics | [KiCad project](v2_pcb/open-race_v2.kicad_pro) |
| [v1_pcb/](v1_pcb/) | Original electronics and JLCPCB exports | [KiCad project](v1_pcb/pcb.kicad_pro) |

## Manufacturing and assembly

- V2 Mini: [production files](v2_mini_pcb/production/), [manual reflow guide](v2_mini_pcb/manual-reflow-guide.md), and [assembly audit](v2_mini_pcb/reflow-assembly-audit.md).
- Full-size V2: [BOM](v2_pcb/BOM.md) and [layout notes](v2_pcb/importantpcb.md).
- V1: [JLCPCB outputs](v1_pcb/jlcpcb/).

The case folder contains only STEP files for import into a CAD editor.

Keep project-local symbol and footprint libraries with their boards: the schematics and layouts reference their library names. KiCad backups, editor history and local state are ignored by the repository.
