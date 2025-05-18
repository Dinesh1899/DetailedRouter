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
    return self._g + self._h
  def __lt__(self, r):
    if self.cost() == r.cost():
      return self._g > r._g
    else:
      return self.cost() < r.cost() 

  def cost1(self):
    return #sum(self._costs)
  
  def __repr__(self):
    return f"({self.x}, {self.y}, {self.layer})"

  def __lt__(self, r):
    if self.cost() == r.cost():
      return self._g > r._g
    else:
      return self.cost() < r.cost()

  def __eq__(self, other):
    return self._id == other._id
    # return self.x == other.x and self.y == other.y and self.layer == other.layer

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
    if u == t:
      # printlog(f"Extracted Target ID: {u._id} XY : {u.x}, {u.y} Layer: {u.layer}, Parent: {u._parent}", True) 
      break
    for vid in u._nbrs:
      v = V[vid]
      newcost = u._g + dist(u, v)
      if newcost < v._g:
        v._g, v._parent = newcost, u
        if v in Q:
          Q.update(v, newcost)
        else:
          # if v == t:
          #   printlog(f"Inserted Target Parent ID: {u._id} XY : {u.x}, {u.y} Layer: {u.layer}, Parent: {u._parent}", True)
          #   printlog(f"Inserted Target ID: {v._id} XY : {v.x}, {v.y} Layer: {v.layer}, Parent: {v._parent}", True)
          Q.push(v)

  # printlog(f" Actual Target ID: {t._id} XY : {t.x}, {t.y} Layer: {t.layer}, Parent: {t._parent}",True)
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
    self._sol = []
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
    
    # print_trdict(tr_dict)
    ## Add source and sink vertices at li1 and met1 layers

    self.add_vertices_to_tracks(tr_dict, vertices)
    self.connect_vertices_on_tracks(tr_dict, vertices)
          
    #plot_vertices(tr_dict, self._name)
    # print_vertices(vertices)
    return tr_dict, vertices

  def connect_vertices_on_tracks(self, tr_dict, vertices):
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

  def add_vertices_to_tracks(self, tr_dict, vertices):
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

  def add_tracks(self, tr_dict, tr_data, reg):
    num, step, start  = tr_data.num, tr_data.step, tr_data.x
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
    trdict, vertices = self.create_graph(layerTrees, tracks)
    srcs = self.get_src_tgt_vertices(trdict, vertices, tracks)
    printlog(f"Source vertices", True)
    print_sources(srcs, vertices)
    ## Call astar for each source and target pair and get the path
    paths = self.getpath(srcs, vertices)
    # print_path(paths)
    self.add_shapes(paths)
    ## Add the path to the net

  def add_shapes(self, paths):
    for path in paths:
      i = 0
      while i < len(path) - 1:
        u = path[i]
        x1, y1 = u.x, u.y
        x2, y2 = u.x, u.y
        i+=1
        v = path[i]
        if layerOrient[u.layer] == 'HORIZONTAL':
          while u.layer == v.layer and u.y == v.y and i < len(path):
            u = v
            x2, y2 = u.x, u.y
            if i == len(path) - 1: break
            i+=1
            v = path[i]
        else:
          while u.layer == v.layer and u.x == v.x and i < len(path):
            u = v
            x2, y2 = u.x, u.y
            if i == len(path) - 1: break           
            i+=1
            v = path[i]

        if x1 == x2 and y1 == y2: continue  
        
        b = layerWidth[u.layer] // 2
        xl, yl = min(x1, x2) - b, min(y1, y2) - b
        xh, yh = max(x1, x2) + b, max(y1, y2) + b
        r = Rect(xl, yl, xh, yh)
        self._sol.append((u.layer, r))
        
        
      # for i in range(len(path)):
      #   u = path[i]
      #   v = path[i+1]
      #   xl = xh = yh = yl = 0
      #   if layerOrient[u.layer] == 'HORIZONTAL':
      #     while u.layer == v.layer and u.y == v.y:
      #       i+=1
      #       v = path[i]

      #   if u.layer == v.layer:
      #     b = layerWidth[u.layer] // 2
      #     xl = min(u.x, v.x) - b
      #     yl = min(u.y, v.y) - b
      #     xh = max(u.x, v.x) + b
      #     yh = max(u.y, v.y) + b
      #     r = Rect(xl, yl, xh, yh)
      #     self._sol.append((u.layer, r))

  def getpath(self, srcs, vertices):
    ## Get the source and target vertices
    paths = list()
    for i in range(len(srcs)):
      for j in range(i+1, len(srcs)):
        s = srcs[i]
        t = srcs[j]
        path = astar(vertices, s, t)
        paths.append(path)
        #print_sources(path, vertices)
    return paths

  def get_src_tgt_vertices(self, trdict, vertices, tracks):
    ## Iterate through all pins of the net
    srcs = list()
    for p, lr in self._pins.items():
      for layer, rects in lr.items():
        ## if layer not in trdict: continue
        rect = [get_bbox(rects)]
        for r in rect:
          ## Get the source and target vertices
          if layerOrient[layer] == 'HORIZONTAL':
            tr_data = tracks[layer][0]
            ny = (r.ur.y - tr_data.x) // tr_data.step
            yp  = tr_data.x + ny * tr_data.step
            tr_vertices = trdict[layer][yp]
            xc = r.ll.x + (r.ur.x - r.ll.x) // 2
            pinv = Vertex(xc, yp, layer, len(vertices))
            vertices.append(pinv)
            min_dist = 10000000
            vid = None
            ## Assume all pins have some overlapping vertices
            for k, v in tr_vertices.items():
              if abs(k - xc) < min_dist:
                min_dist = abs(k - xc)
                vid = v
            pinv._nbrs.append(vid)
            vertices[vid]._nbrs.append(pinv._id)
            srcs.append(vertices[pinv._id])
          else:
            tr_data = tracks[layer][1]
            nx = (r.ur.x - tr_data.x) // tr_data.step
            xp  = tr_data.x + nx * tr_data.step
            # printlog(f"XP: {xp} Rect: {r.ll.x}, {r.ll.y}, {r.ur.x}, {r.ur.y} Layer: {layer} Tracks: {trdict[layer].keys()}", True)
            if xp not in trdict[layer]:
              xp = min(trdict[layer].keys())
            tr_vertices = trdict[layer][xp]
            yc = r.ll.y + (r.ur.y - r.ll.y) // 2
            pinv = Vertex(xp, yc, layer, len(vertices))
            vertices.append(pinv)
            min_dist = 10000000
            vid = None
            ## Assume all pins have some overlapping vertices
            for k, v in tr_vertices.items():
              if abs(k - yc) < min_dist:
                min_dist = abs(k - yc)
                vid = v
            pinv._nbrs.append(vid)
            vertices[vid]._nbrs.append(pinv._id)
            srcs.append(vertices[pinv._id])
    return srcs     

def get_bbox(rects):
  xl = yl = xh = yh = 0
  for r in rects:
    if xl == 0 and yl == 0 and xh == 0 and yh == 0:
      xl, yl = r.ll.x, r.ll.y
      xh, yh = r.ur.x, r.ur.y
    else:
      xl, yl = min(xl, r.ll.x), min(yl, r.ll.y)
      xh, yh = max(xh, r.ur.x), max(yh, r.ur.y)
  return Rect(xl, yl, xh, yh)


def print_trdict(tr_dict):
  for layer in tr_dict:
    printlog(f"Layer: {layer}, DIR:{layerOrient[layer]}", True)
    track_info = ""
    for tr in tr_dict[layer]:
      track_info += f" {tr}"
    printlog(f"  Tracks: {track_info}", True)  

def print_path(paths):
  printlog("Paths:", True)
  for path in paths:
    printlog("  Path:", True)
    for v in path:
      printlog(f"    Vertex: {v.x}, {v.y} Layer: {v.layer}", True)



def print_sources(srcs, vertices):
  for v in srcs:
    printlog(f"ID: {v._id} Vertex: {v.x}, {v.y} Layer: {v.layer}", True)
    for nbr in v._nbrs:
      printlog(f"  NBR ID: {nbr} Vertex: {vertices[nbr].x}, {vertices[nbr].y} Layer: {vertices[nbr].layer}", True)

def print_vertices(vertices):
  for v in vertices:
    printlog(f"ID: {v._id} Vertex: {v.x}, {v.y} Layer: {v.layer}", True)
    for nbr in v._nbrs:
      printlog(f"  NBR ID: {nbr} Vertex: {vertices[nbr].x}, {vertices[nbr].y} Layer: {vertices[nbr].layer}", True)

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


def get_components(ideff, leff):
  # leff = LEFDEFParser.LEFReader()
  # ideff = LEFDEFParser.DEFReader()
  # odeff = LEFDEFParser.DEFReader()
  # leff.readLEF(lef)
  # ideff.readDEF(idef)
  # odeff.readDEF(odef)

  lefDict = {m.name() : m for m in leff.macros()}

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
  netDict = dict()
  idx = 0
  for net in ideff.nets():
    if net.name() not in skipNets:
      nets.append(Net(net, insts, pins, idx))
      netDict[net.name()] = net
      idx += 1

  # for net in odeff.nets():
  #   if net.name() in netDict:
  #     netDict[net.name()]._sol = net.rects()

  obsts = dict()
  markUnusedPins(nets, insts, pins, obsts)
  return [nets, insts, pins, obsts]
  # bbox = ideff.bbox()

def sort_nets(nets):
  for net in nets:
    net.calculate_bbox()
  
  nets.sort(key=lambda n: n.hpwl())

def printnets(nets):
  for net in nets:
    printlog(f"Net: {net._name} ID: {net._id} HPWL: {net.hpwl()} BBox: {net._bbox.ll.x}, {net._bbox.ll.y}, {net._bbox.ur.x}, {net._bbox.ur.y}", True)
    for p, lr in net._pins.items():
      printlog(f"  Pin: {p[1]} Type: {p[0]}", True)
      for layer, rects in lr.items():
        for r in rects:
          printlog(f"    Layer:{layer} Rect: {r.ll.x}, {r.ll.y}, {r.ur.x}, {r.ur.y}", True)



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


def add_net_shapes(net, netDEF):
  for r in net._sol:
    layer, rect = r
    netDEF.addRect(layer, rect.ll.x, rect.ll.y, rect.ur.x, rect.ur.y)  

def route_nets(nets: list[Net], layerTrees, tracks):
  ids = [4] # 1: N1_d, 8:N3, 9:N3_d, 15: net1, 2: _02_
  for net in nets:
    if net._id in ids:
      printlog(f"Routing net: {net._name} ID: {net._id}", True)
      net.route(layerTrees, tracks)

def writeDEF(netDict, ideff, odef):
  names = ["_04_"]

  # for name in netDict:
  #   if len(netDict[name]._pins.keys()) > 2:
  #     print(f"Net: {name}") 


  for inet in ideff.nets():
    if inet.name() in names:
      if inet.name() not in skipNets:
        shapes = netDict[inet.name()]._sol
        for r in shapes:
          layer, rect = r
          inet.addRect(layer, rect.ll.x, rect.ll.y, rect.ur.x, rect.ur.y)

  ideff.writeDEF(odef)



def plot_rectangles(color='blue', alpha=0.5, title="Rectangles"):
  import matplotlib.pyplot as plt
  import matplotlib.patches as patches

  rects = []
  rects.append(Rect(7925, 28760, 8195, 29665))
  rects.append(Rect(8025, 27960, 8195, 28760))
  rects.append(Rect(7935, 27455, 8195, 27960))



  """
  Plot a list of rectangles
  Args:
    rects: List of Rect objects with ll (lower left) and ur (upper right) coordinates
    color: Color for the rectangles
    alpha: Transparency value (0-1)
    title: Plot title
  """
  fig, ax = plt.subplots()
  
  # Plot each rectangle
  for rect in rects:
    width = rect.ur.x - rect.ll.x
    height = rect.ur.y - rect.ll.y
    
    rect_patch = patches.Rectangle(
      (rect.ll.x, rect.ll.y),
      width, height,
      facecolor=color,
      alpha=alpha
    )
    ax.add_patch(rect_patch)
  
  # Update plot limits to show all rectangles
  ax.autoscale()
  
  # Add grid and title
  ax.grid(True)
  ax.set_title(title)
  
  plt.show()


## Detail Router
def detailed_route(idef, ilef, guide, odef):
  # Init Insts, Nets and Pins from LEF/DEF Files  
  leff = LEFDEFParser.LEFReader()
  ideff = LEFDEFParser.DEFReader()
  odeff = LEFDEFParser.DEFReader()
  leff.readLEF(ilef)
  ideff.readDEF(idef)
  # odeff.readDEF(odef)  

  nets, insts, pins, obsts = get_components(ideff, leff)
  tracks = ideff.tracks()# get_tracks(idef)

  netDict = dict()
  for net in nets:
    netDict[net._name] = net
  
  sort_nets(nets)
  printnets(nets)
  parse_guides(netDict, guide)
  # printguides(nets)
  
  layerTrees = buildTree(nets, insts, obsts)
  route_nets(nets, layerTrees, tracks)
  writeDEF(netDict, ideff, odef)
  # plot_rectangles()
  return None


if __name__ == "__main__":
  # Example usage
  ckt = "add5"
  idef = f"def/{ckt}.def"
  ilef = f"lef/sky130.lef"
  guide = f"gr/{ckt}.guide"
  odef = f"sol/{ckt}_out.def"    
  VERBOSE = True
  LOG_FILE = f"router_{ckt}.log"
  printlog(f"Start routing {ckt}", True)
  printlog(f"                     ", True)
  printlog(f"                     ", True)
  printlog(f"                     ", True)
  detailed_route(idef, ilef, guide, odef)
  
  from checker import loadAndCheck
  loadAndCheck(odef, idef, ilef, False)
