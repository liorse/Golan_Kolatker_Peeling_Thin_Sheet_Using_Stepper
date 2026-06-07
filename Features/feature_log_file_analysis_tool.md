# Log File Analysis Tool

## Goal

A standalone HTML/JS tool for analysing peeling experiment CSV log files.

## Data Source

Log files are at `/home/liors/.config/peeling-controller/logs/`.

Two CSV schemas exist (tool handles both):
- **Older:** `timestamp, time_ms, pos_um, speed_um_s, temp_c, state`
- **Newer:** `timestamp, time_ms, pos_um, speed_um_s, temp_c, heater_duty, state`

## Design Decisions

| # | Decision | Choice |
|---|---|---|
| 1 | X-axis | Wall-clock timestamp (`timestamp` column) |
| 2 | Multi-file display | One chart, all selected runs overlaid with distinct colors |
| 3 | Duty cycle display | Text label in legend per run — e.g. "Run 14:10 — duty 255 (100%)" |
| 4 | File selection | Folder picker → checklist of CSVs to include |
| 5 | Chart library | Chart.js from CDN |
| 6 | Output location | `analysis_folder/analysis.html` |

## Behaviour

- User clicks "Open Folder", picks the logs folder
- A checklist of all `.csv` files appears (sorted newest-first)
- User checks the runs they want and clicks "Plot"
- A single temperature-vs-time chart renders with one colored line per run
- Each legend entry shows: filename stem + duty cycle if available (e.g. "14:10 — duty 100%")
- Files missing `heater_duty` show "duty n/a" in the legend
