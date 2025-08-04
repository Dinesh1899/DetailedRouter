# DetailedRouter
Course Project for EE5333 : Physical Design Automation at IIT Madras

## Project Setup

1. Clone the repository and set up environment:
```bash
git clone <repository-url>
cd DetailRouter
chmod +x setup.sh
./setup.sh
```

The setup script installs required dependencies:
- Python 3.8+
- matplotlib
- rtree spatial indexing library 
- LEFDEFParser library

## Input Files Required

The router requires 3 input files:

1. **LEF File** (.lef)
   - Contains layer information (width, spacing)
   - Standard cell definitions
   - Pin locations within cells

2. **DEF File** (.def) 
   - Placement information
   - Net connections
   - Routing tracks
   - Pin locations

3. **Guide File** (.guide)
   - Output from global router
   - Contains guide rectangles for each net
   - Format: `x1 y1 x2 y2 layer`

## Internal Data Structures

### Instance (Inst) Class
- Stores transformed cell coordinates
- Contains pin shapes and obstruction shapes
- Tracks used/unused pins

### Net Class
- Stores net name, ID and connected pins
- Maintains guide rectangles from global router
- Contains final routed solution

### R-tree Structure
- Spatial index for efficient shape queries
- Separate trees for each routing layer
- Used to store:
  - Cell obstructions
  - Pin shapes  
  - Routed wire segments

## Routing Algorithm 

1. **Net Ordering**
   - Calculate HPWL (Half-Perimeter Wirelength) for each net
   - Sort nets in increasing order of HPWL
   - Process smaller nets first

2. **Shape Filtering**
   - For each net, create filtered r-tree containing only:
     - Shapes intersecting with guide rectangles
     - Used for efficient obstacle queries

3. **Graph Construction**
   - Build grid graph from routing tracks
   - Vertices at track intersections
   - Edges between adjacent track intersections
   - Via edges between layers

4. **Path Finding**
   - Use Prim's MST algorithm to find minimum spanning tree connecting pins
   - For each MST edge:
     - Use A* algorithm to find detailed routing path
     - Consider via costs and obstacle penalties

## Running the Router

Basic usage:
```bash
python3 router.py -l <lef_file> -i <input_def> -g <guide_file> -o <output_def>
```

Example:
```bash
python3 router.py -l lef/sky130.lef -i def/c17.def -g gr/c17.guide -o outputs/c17_out.def
```

Logs are stored in: `logs/router_<def_name>.log`

## Design Rule Checking

To verify the routed solution:
```bash 
python3 checker.py -l <lef_file> -i <input_def> -o <output_def> [-p]
```
Example usage:
```bash
python3 checker.py -l lef/sky130.lef -i def/c17.def -o outputs/c17_out.def
```

Note:
- Check <output_def> got generated or not after running the router.py before running the checker

Options:
- `-p`: Generate visualization plots

Example with plotting:
```bash
python3 checker.py -l lef/sky130.lef -i def/c17.def -o outputs/c17_out.def -p
```

## Output Verification

1. Check output DEF files in `outputs/` directory
2. Review DRC violation logs 
3. Examine visualization plots if `-p` option used
4. Verify nets are fully connected
5. Check for any spacing violations

The checker helps ensure:
- All nets are properly routed
- No DRC violations
- Pins are correctly connected
- No shorts between nets

## Visualization

When using the `-p` option with checker.py:
- Generates layer-by-layer routing plots
- Shows:
  - Layerwise tracks and metal shapes
  - DRC violations (if any)

## Debug Logging

To enable detailed debug logs:
1. Open router.py
2. Find line containing `VERBOSE = False` (around line 994)
3. Change it to `VERBOSE = True`
4. Uncomment the required printlog and other print methods
5. Make sure the verbose argument(2nd argument) of all the desired printlog statements is set to `True` 
6. Logs will be generated in `logs/router_<def_name>.log`

## Running Test Examples

The repository includes example circuits in `def/` directory along with global routed guides in `guide/` directory. You can run all examples using the provided shell scripts:

1. Run detailed router on all examples:
```bash
chmod +x route.sh
./route.sh
```
2. Check DRC and connectivity for all routed solutions:
```bash
chmod +x check.sh
./check.sh
```
The scripts will:
- Route all the example circuits using router.py
- Generate output DEF files in outputs/ directory
- Run DRC/connectivity checks on all the def files generated for all the examples using checker.py

