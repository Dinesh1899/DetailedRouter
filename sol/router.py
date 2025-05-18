from typing import OrderedDict
import LEFDEFParser
from LEFDEFParser import Rect
import rtree
import heapq as hq

import datetime

# skip decap, fill and tap cells
skipCells = {"sky130_fd_sc_hd__decap_3", "sky130_fd_sc_hd__decap_4", "sky130_fd_sc_hd__decap_6", "sky130_fd_sc_hd__decap_8",\
            "sky130_fd_sc_hd__decap_12", "sky130_fd_sc_hd__fill_1", "sky130_fd_sc_hd__fill_2", "sky130_fd_sc_hd__fill_4",
            "sky130_fd_sc_hd__fill_8", "sky130_fd_sc_hd__lpflow_decapkapwr_3", "sky130_fd_sc_hd__lpflow_decapkapwr_4",\
            "sky130_fd_sc_hd__lpflow_decapkapwr_6", "sky130_fd_sc_hd__lpflow_decapkapwr_8", "sky130_fd_sc_hd__lpflow_decapkapwr_12", \
            "sky130_fd_sc_hd__lpflow_lsbuf_lh_hl_isowell_tap_1", "sky130_fd_sc_hd__lpflow_lsbuf_lh_hl_isowell_tap_2", \
            "sky130_fd_sc_hd__lpflow_lsbuf_lh_hl_isowell_tap_4", "sky130_fd_sc_hd__lpflow_lsbuf_lh_isowell_tap_1", \
            "sky130_fd_sc_hd__lpflow_lsbuf_lh_isowell_tap_2", "sky130_fd_sc_hd__lpflow_lsbuf_lh_isowell_tap_4", "sky130_fd_sc_hd__tap_1", \
            "sky130_fd_sc_hd__tap_2", "sky130_fd_sc_hd__tapvgnd2_1", "sky130_fd_sc_hd__tapvgnd_1", \
            "sky130_fd_sc_hd__tapvpwrvgnd_1", "sky130_ef_sc_hd__decap_12"}

layerColors = { 'li1': 'red', 'met1': 'blue', 'met2': 'green', 'met3': 'orange', 'met4': 'magenta', 'met5': 'cyan' }
layerOrient = { 'li1': 'VERTICAL', 'met1': 'HORIZONTAL', 'met2': 'VERTICAL', 'met3': 'HORIZONTAL', 'met4': 'VERTICAL', 'met5': 'HORIZONTAL' }

layerNum = {'li1': 1,'met1': 2,'met2': 3,'met3': 4,'met4': 5,'met5': 6}

# skip power/ground/clock nets
skipNets = {'clk', 'VPWR', 'VGND'}

adjLayer = {
  'li1':  ['met1'],
  'met1': ['li1',  'met2'],
  'met2': ['met1', 'met3'],
  'met3': ['met2', 'met4'],
  'met4': ['met3', 'met5'],
  'met5': ['met4']
}

layerWidth = dict()
layerSpacing = {'li1': 170, 'met1': 140, 'met2': 140, 'met3': 300, 'met4': 300, 'met5': 1600}

VERBOSE = False
LOG_FILE = "router_debug.log"
import datetime


class Vertex:
  def __init__(self, x, y, layer, id, parent=None, nbrs=None):
    self._id = id
    self.x = x
    self.y = y
    self.layer = layer
    self._parent = parent
    self._nbrs = []
    self._g = None
    self._h = None
    self._costs = []

  def cost(self):
    return sum(self._costs)
  
  def __repr__(self):
    return f"({self.x}, {self.y}, {self.layer})"

  def __lt__(self, r):
    return self.cost() < r.cost()
    # if self.cost() == r.cost():
    #   return self._g > r._g
    # else:
    #   return self.cost() < r.cost()

  def __eq__(self, other):
    return self.x == other.x and self.y == other.y and self.layer == other.layer

  def __hash__(self):
    return hash((self.x, self.y, self.layer))

def dist(u, v):
  return abs(u.x - v.x) + abs(u.y - v.y) + abs(layerNum[u.layer] - layerNum[v.layer])

class priority_queue:
  def __init__(self, vertices = []):
    self._vertices = vertices[:]
    self._q = vertices[:]
    hq.heapify(self._q)
  def push(self, v):
    hq.heappush(self._q, v)
  def pop(self):
    return(hq.heappop(self._q))
  def update(self, v, g):
    try: i = self._q.index(v)
    except ValueError: i = None
    if i is not None:
      self._q[i]._g = g
      hq.heapify(self._q)
  def updateIndex(self, i, g):
    assert i < len(self._q)
    self._vertices[i]._g = g
    hq.heapify(self._q)
  def empty(self):
    return len(self._q) == 0
  def __contains__(self, v):
    return v in self._q
  def __repr__(self):
    return str(self._q)

def astar(V, s, t):
  for v in V:
    v._g, v._h, v._parent = 10000000, dist(v,t), None
  s._g = 0
  s._h = dist(s,t)
  Q = priority_queue()
  Q.push(s)
  
  while not Q.empty():
    u = Q.pop()
    if u == t: break
    for v in u._nbrs:
      newcost = u._g + dist(u, v)
      if newcost < v._g:
        v._g, v._parent = newcost, u
        if v in Q:
          Q.update(v, newcost)
        else:
          Q.push(v)
  
  path = [t]
  while path[-1]._parent is not None:
    path.append(path[-1]._parent)
  return path

def printlog(str, print=False):
  timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
  logline = f"{timestamp} {str}"
  if VERBOSE and print:
    # print(logline)
    with open(LOG_FILE, "a") as f:
      f.write(logline + "\n")


# instance from the component list that is transformed using the orientation and origin
class Inst:
  def __init__(self, comp, macro):
    self._comp = comp
    self._macro = macro
    origin = comp.location()
    self._bbox = Rect(origin.x, origin.y, origin.x + macro.xdim(), origin.y + macro.ydim())
    self._pins = dict()
    self._markpins = dict()
    self._obsts = dict()
    for p in macro.pins():
      shapes = dict()
      for port in p.ports():
        for layer, rects in port.items():
          if layer not in layerColors: continue
          if layer not in shapes: shapes[layer] = list()
          for v in rects:
            r = Rect(v.ll.x, v.ll.y, v.ur.x, v.ur.y)
            r.transform(comp.orient(), origin, macro.xdim(), macro.ydim())
            shapes[layer].append(r)
      self._pins[p.name()] = shapes
      self._markpins[p.name()] = False

    for layer, rects in macro.obstructions().items():
      if layer not in layerColors: continue
      if layer not in self._obsts: self._obsts[layer] = list()
      for v in rects:
        r = Rect(v.ll.x, v.ll.y, v.ur.x, v.ur.y)
        r.transform(comp.orient(), origin, macro.xdim(), macro.ydim())
        self._obsts[layer].append(r)

def plot_vertices(tr_dict, net_name):
  """Plot vertices for each layer in tr_dict"""
  import matplotlib.pyplot as plt
  fig, axs = plt.subplots(2, 3, figsize=(15, 10))
  fig.suptitle(f'Vertices for net {net_name}')
  
  # Map layers to subplot positions
  layer_to_pos = {
    'li1': (0, 0),
    'met1': (0, 1),
    'met2': (0, 2),
    'met3': (1, 0),
    'met4': (1, 1),
    'met5': (1, 2)
  }
  
  # Plot vertices for each layer
  for layer in layerColors:
    if layer not in tr_dict:
      continue
        
    row, col = layer_to_pos[layer]
    ax = axs[row, col]
    
    # Plot vertices
    x_coords = []
    y_coords = []
    for track_pos, vertices in tr_dict[layer].items():
      for vertex in vertices:
        x_coords.append(vertex.x)
        y_coords.append(vertex.y)
    
    ax.scatter(x_coords, y_coords, c=layerColors[layer], label=f'{layer} vertices')
    ax.set_title(f'Layer {layer}')
    ax.grid(True)
    ax.legend()
  
  plt.tight_layout()
  plt.savefig(f'ref/vertices_{net_name}.png')
  plt.close()


# wrapper Net class that has the actual pins at the transformed coordinates
class Net:
  def __init__(self, net, insts, pins, idx):
    self._name = net.name()
    self._pins = dict()
    self._id   = idx # unique ID used to identify nets from rtree
    self._bbox = Rect(0, 0, 0, 0) ## Largest Rectangle that contains all the pins
    self._guides = dict()
    for p in net.pins():
      if p[0] in insts:
        self._pins[p] = insts[p[0]]._pins[p[1]] # copy shapes from the transformed instance pins
      elif p[0] == 'PIN' and p[1] in pins:
        self._pins[p] = pins[p[1]] # copy shapes from the boundary pins

  def calculate_bbox(self):
    xl = yl = xh = yh = 0
    for p, lr in self._pins.items():
      for layer, rects in lr.items():
        if layer not in layerColors: continue
        for r in rects:
          if xl == 0 and yl == 0 and xh == 0 and yh == 0:
            xl = r.ll.x
            yl = r.ll.y
            xh = r.ur.x
            yh = r.ur.y
          else:
            xl = min(xl, r.ll.x)
            yl = min(yl, r.ll.y)
            xh = max(xh, r.ur.x)
            yh = max(yh, r.ur.y)
    self._bbox = Rect(xl, yl, xh, yh)

  def hpwl(self):
    return self._bbox.ur.x - self._bbox.ll.x + self._bbox.ur.y - self._bbox.ll.y

  def create_graph(self, layerTrees, tracks):
    # Create a graph representation of the net
    tr_dict = dict()
    vertices = list()
    for layer, regions in self._guides.items():
      t = 0 if layerOrient[layer] == 'HORIZONTAL' else 1
      tr_data = tracks[layer][t]
      if layer not in tr_dict:
        tr_dict[layer] = dict()
      for reg in regions:
        self.add_tracks(tr_dict[layer], tr_data, reg)
    
    ## Add source and sink vertices at li1 and met1 layers
    for layer in tr_dict:
      printlog(f"Layer: {layer}, DIR:{layerOrient[layer]}", True)
      track_info = ""
      for tr in tr_dict[layer]:
        track_info += f" {tr}"
      printlog(f"  Tracks: {track_info}", True)

    for layer in layerColors:
      ## Add via edges with the layer above
      if layer not in tr_dict: break
      adj_lr = adjLayer[layer][1] if len(adjLayer[layer]) > 1 else adjLayer[layer][0]
      if adj_lr not in tr_dict: break
      for ct in tr_dict[layer]:
        for atr in tr_dict[adj_lr]:
          if layerOrient[layer] == 'HORIZONTAL':
            if atr in tr_dict[layer][ct]:
              cv = vertices[tr_dict[layer][ct][atr]]
              av = Vertex(atr, ct, adj_lr, len(vertices))
              vertices.append(av)
              cv._nbrs.append(av._id)
              av._nbrs.append(cv._id)
              tr_dict[adj_lr][atr][ct] = av._id
            else:
              cv = Vertex(atr, ct, layer, len(vertices))
              vertices.append(cv)
              av = Vertex(atr, ct, adj_lr, len(vertices))
              vertices.append(av)
              cv._nbrs.append(av._id)
              av._nbrs.append(cv._id)
              tr_dict[layer][ct][atr] = cv._id
              tr_dict[adj_lr][atr][ct] = av._id
          else:
            if atr in tr_dict[layer][ct]:
              cv = vertices[tr_dict[layer][ct][atr]]
              av = Vertex(ct, atr, adj_lr, len(vertices))
              vertices.append(av)
              cv._nbrs.append(av._id)
              av._nbrs.append(cv._id)
              tr_dict[adj_lr][atr][ct] = av._id
            else:
              cv = Vertex(ct, atr, layer, len(vertices))
              vertices.append(cv)
              av = Vertex(ct, atr, adj_lr, len(vertices))
              vertices.append(av)
              cv._nbrs.append(av._id)
              av._nbrs.append(cv._id)
              tr_dict[layer][ct][atr] = cv._id
              tr_dict[adj_lr][atr][ct] = av._id          

    for layer in layerColors:
      ## Add via edges with the layer above
      if layer not in tr_dict: break
      for ct in tr_dict[layer]:
        tr_vertices = [v for k,v in sorted(tr_dict[layer][ct].items())]
        for i,v in enumerate(tr_vertices):
          if i == 0:
            vertices[v]._nbrs.append(tr_vertices[i+1])
          elif i == len(tr_vertices) - 1:
            vertices[v]._nbrs.append(tr_vertices[i-1])
          else:
            vertices[v]._nbrs.append(tr_vertices[i+1])
            vertices[v]._nbrs.append(tr_vertices[i-1])

    #plot_vertices(tr_dict, self._name)
    for v in vertices:
      printlog(f"ID: {v._id} Vertex: {v.x}, {v.y} Layer: {v.layer}", True)
      for nbr in v._nbrs:
        printlog(f"  NBR ID: {nbr} Vertex: {vertices[nbr].x}, {vertices[nbr].y} Layer: {vertices[nbr].layer}", True)
              

  def add_tracks(self, tr_dict, tr_data, reg):
    num, step, start  = tr_data.num, tr_data.step*10, tr_data.x
    dir = tr_data.orient
    lo = hi = 0
    if dir == 'X':
      lo, hi = reg.ll.y, reg.ur.y
    else:
      lo, hi = reg.ll.x, reg.ur.x
    
    pl = int((lo - start) / step)
    lo = pl + 1  if lo > start else 0
    ph = int((hi - start) / step)
    hi = ph if hi > start else 0

    for i in range(lo, hi+1):
      tr_dict[start + i*step] = dict()




  def route(self, layerTrees, tracks):
    # Construct graph based on the guides and tracks information
    self.create_graph(layerTrees, tracks)


def markUnusedPins(nets, insts, pins, obsts):
  markpins = {k:False for k in pins}
  for net in nets:
    for p in net._pins:
      if p[0] in insts:
        insts[p[0]]._markpins[p[1]] = True
      elif p[0] == 'PIN' and p[1] in markpins:
        markpins[p[1]] = True
  for p in pins:
    if not markpins[p]:
      for l, rects in pins[p].items():
        if l not in obsts: obsts[l] = list()
        for r in rects: obsts[l].append(r)

  for nm, inst in insts.items():
    for k, v in inst._markpins.items():
      if not v:
        for l, rects in inst._pins[k].items():
          if l not in obsts: obsts[l] = list()
          for r in rects: obsts[l].append(r)

def buildTree(nets, insts, obsts):
  lT = {layer: rtree.index.Index() for layer in layerColors}
  obstid = len(nets)

  count = 0
  for inst in insts.values():
    for layer, rects in inst._obsts.items():
      for r in rects:
        lT[layer].insert(count, (r.ll.x, r.ll.y, r.ur.x, r.ur.y), obj=obstid)
        count += 1

  for layer, rects in obsts.items():
    for r in rects:
      lT[layer].insert(count, (r.ll.x, r.ll.y, r.ur.x, r.ur.y), obj=obstid)
      count += 1

  for net in nets:
    for p, lr in net._pins.items():
      for layer, rects in lr.items():
        for r in rects:
          lT[layer].insert(count, (r.ll.x, r.ll.y, r.ur.x, r.ur.y), obj=net._id)
          count += 1

  return lT


def get_components(odef, idef, lef):
  leff = LEFDEFParser.LEFReader()
  ideff = LEFDEFParser.DEFReader()
  odeff = LEFDEFParser.DEFReader()
  leff.readLEF(lef)
  lefDict = {m.name() : m for m in leff.macros()}
  ideff.readDEF(idef)
  odeff.readDEF(odef)

  for layer in leff.layers():
    layerWidth[layer.name()] = layer.width()

  insts = {inst.name():Inst(inst, lefDict[inst.macro()]) for inst in ideff.components() if inst.macro() not in skipCells}

  pins = dict()
  markpins = dict()
  for p in ideff.pins():
    pn = p.name()
    pins[pn] = dict()
    for port in p.ports():
      for layer, rects in port.items():
        if layer not in pins[pn]: pins[pn][layer] = list()
        for r in rects:
          pins[pn][layer].append(Rect(r.ll.x, r.ll.y, r.ur.x, r.ur.y))

  nets = list()
  idx = 0
  for net in ideff.nets():
    if net.name() not in skipNets:
      nets.append(Net(net, insts, pins, idx))
      idx += 1

  # for net in odeff.nets():
  #   if net.name() in netDict:
  #     netDict[net.name()]._sol = net.rects()

  obsts = dict()
  markUnusedPins(nets, insts, pins, obsts)
  return [nets, insts, pins, obsts]
  # bbox = ideff.bbox()

def get_tracks(idef):
  ideff = LEFDEFParser.DEFReader()
  ideff.readDEF(idef)
  return ideff.tracks()

def sort_nets(nets):
  for net in nets:
    net.calculate_bbox()
  
  nets.sort(key=lambda n: n.hpwl())

def printnets(nets):
  for net in nets:
    printlog(f"Net: {net._name} ID: {net._id} HPWL: {net.hpwl()} BBox: {net._bbox.ll.x}, {net._bbox.ll.y}, {net._bbox.ur.x}, {net._bbox.ur.y}", True)
    # for p, lr in net._pins.items():
    #   print(f"  Pin: {p[1]} Layer: {p[0]}")
    #   for layer, rects in lr.items():
    #     for r in rects:
    #       print(f"    Rect: {r.ll.x}, {r.ll.y}, {r.ur.x}, {r.ur.y}")



def parse_guides(netDict, guide):
  with open(guide, 'r') as f:
    current_net = None
    in_net = False
    for line in f:
      line = line.strip()
      if not line or line.startswith("#"):
        continue
      # Net name line (next line should be '(')
      if not in_net and not line.endswith('(') and not line == ')':
        current_net = line
        continue
      # Start of net section
      if line == '(':
        in_net = True
        continue
      # End of net section
      if line == ')':
        current_net = None
        in_net = False
        continue
      # Guide line inside net section
      if in_net and current_net:
        parts = line.split()
        if len(parts) != 5:
          continue
        x1, y1, x2, y2, layer = parts
        printlog(f"Guide net: {current_net}", False)
        if current_net not in netDict:
          continue
        if layer not in netDict[current_net]._guides:
          netDict[current_net]._guides[layer] = []
        netDict[current_net]._guides[layer].append(Rect(int(x1), int(y1), int(x2), int(y2)))


def printguides(nets):
  for net in nets:
    printlog(f"Net: {net._name} ID: {net._id}", False)
    for layer, rects in net._guides.items():
      printlog(f"  Layer: {layer}", False)
      for r in rects:
        printlog(f"    Rect: {r.ll.x}, {r.ll.y}, {r.ur.x}, {r.ur.y}", False)

def route_nets(nets: list[Net], layerTrees, tracks):
  ids = [1]
  for net in nets:
    if net._id in ids:
      printlog(f"Routing net: {net._name} ID: {net._id}", True)
      net.route(layerTrees, tracks)

## Detail Router
def detailed_route(idef, ilef, guide, odef):
  # Init Insts, Nets and Pins from LEF/DEF Files
  nets, insts, pins, obsts = get_components(idef, idef, ilef)
  tracks = get_tracks(idef)
  netDict = dict()
  for net in nets:
    netDict[net._name] = net
  
  sort_nets(nets)
  # printnets(nets)
  parse_guides(netDict, guide)
  # printguides(nets)
  
  layerTrees = buildTree(nets, insts, obsts)
  route_nets(nets, layerTrees, tracks)

  return None


if __name__ == "__main__":
  # Example usage
  ckt = "c17"
  idef = f"def/{ckt}.def"
  ilef = f"lef/sky130.lef"
  guide = f"gr/{ckt}.guide"
  odef = f"def/{ckt}_out.def"    
  VERBOSE = True
  LOG_FILE = f"router_{ckt}.log"
  printlog(f"Start routing {ckt}", True)
  detailed_route(idef, ilef, guide, odef)
