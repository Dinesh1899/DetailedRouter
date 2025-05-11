import LEFDEFParser
import rtree

## Detail Router
def detailed_route(idef, ilef, guide, odef):   
  return None


if __name__ == "__main__":
  # Example usage
  idef = "../def/c17.def"
  ilef = "../lef/c17.lef"
  guide = "../guide/c17.guide"
  odef = "../def/c17_out.def"    
  
  detailed_route(idef, ilef, guide, odef)
