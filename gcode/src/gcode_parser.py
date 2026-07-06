#!/usr/bin/env python3
import re, sys, math, zipfile, argparse, csv

FIRST_LAYER_Z = 0.4
Z_CLAMP       = 0.01
SIMPLIFY_TOL  = 0.005
CROSS_NORM_MIN = 2.5e-5

WORD_RE = re.compile(r"([A-Z])\s*([-+]?\d*\.?\d+)")

def load_reach_bounds(csv_path):
    pts = set()
    xs_all, ys_all = set(), set()
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            lx = round(float(row["local_x"]) * 1000.0, 3)
            ly = round(float(row["local_y"]) * 1000.0, 3)
            pts.add((lx, ly))
            xs_all.add(lx)
            ys_all.add(ly)
    if not pts:
        raise ValueError(f"no reachable points in {csv_path}")

    xs = sorted(xs_all)
    ys = sorted(ys_all)
    nx, ny = len(xs), len(ys)
    xi = {x: i for i, x in enumerate(xs)}
    yi = {y: i for i, y in enumerate(ys)}

    # build occupancy grid: True if reachable
    grid = [[False]*nx for _ in range(ny)]
    for (lx, ly) in pts:
        grid[yi[ly]][xi[lx]] = True

    # largest rectangle in histogram approach
    # for each row, compute height (consecutive reachable cells upward)
    height = [[0]*nx for _ in range(ny)]
    for col in range(nx):
        for row in range(ny):
            if grid[row][col]:
                height[row][col] = 1 if row == 0 else height[row-1][col] + 1
            else:
                height[row][col] = 0

    best_area = 0
    best_rect = (0, 0, 0, 0)  # col_left, col_right, row_bottom, row_top

    for row in range(ny):
        # largest rectangle in histogram for this row
        stack = []
        for col in range(nx + 1):
            h = height[row][col] if col < nx else 0
            while stack and height[row][stack[-1]] > h:
                top = stack.pop()
                w = col if not stack else col - stack[-1] - 1
                ht = height[row][top]
                area = w * ht
                if area > best_area:
                    best_area = area
                    left = 0 if not stack else stack[-1] + 1
                    right = col - 1
                    best_rect = (left, right, row - ht + 1, row)
            stack.append(col)

    cl, cr, rt, rb = best_rect
    x_min, x_max = xs[cl], xs[cr]   
    y_min, y_max = ys[rt], ys[rb]

    rect_w = x_max - x_min
    rect_h = y_max - y_min
    print(f"largest inscribed rectangle: {rect_w:.1f} x {rect_h:.1f} mm")
    print(f"  X[{x_min:.1f}, {x_max:.1f}] Y[{y_min:.1f}, {y_max:.1f}]")
    print(f"  grid cells: cols[{cl},{cr}] rows[{rt},{rb}], area={best_area} cells")

    return x_min, x_max, y_min, y_max

def load_lines(path):
    if path.lower().endswith(".3mf"):
        with zipfile.ZipFile(path, "r") as z:
            for name in z.namelist():
                if name.lower().endswith(".gcode"):
                    return z.read(name).decode("utf-8", errors="ignore").splitlines()
        raise FileNotFoundError(f"no .gcode inside {path}")
    return open(path, errors="ignore").read().splitlines()

def parse_moves(lines):
    absolute_xyz = True; absolute_e = True
    cur = {"X":0.0,"Y":0.0,"Z":0.0,"E":0.0,"F":0.0}
    moves=[]
    for raw in lines:
        line = re.sub(r"\(.*?\)","",raw.split(";")[0]).strip().upper()
        if not line: continue
        words = dict((k,float(v)) for k,v in WORD_RE.findall(line))
        if "M" in words and "G" not in words:
            m=int(words["M"])
            if m==82: absolute_e=True
            if m==83: absolute_e=False
            continue
        if "G" not in words: continue
        g=int(words["G"])
        if g==90: absolute_xyz=True; continue
        if g==91: absolute_xyz=False; continue
        if g==92:
            for k in ("X","Y","Z","E"):
                if k in words: cur[k]=words[k]
            continue
        if g not in (0,1,2,3): continue
        prev_e=cur["E"]
        for k in ("X","Y","Z"):
            if k in words: cur[k]=words[k] if absolute_xyz else cur[k]+words[k]
        if "E" in words: cur["E"]=words["E"] if absolute_e else cur["E"]+words["E"]
        if "F" in words: cur["F"]=words["F"]
        if not any(k in words for k in ("X","Y","Z","I","J")): continue
        mv={"cmd":"G1" if g==0 else f"G{g}","X":cur["X"],"Y":cur["Y"],"Z":cur["Z"],
            "E":cur["E"],"dE":cur["E"]-prev_e,"F":cur["F"]}
        if "I" in words: mv["I"]=words["I"]
        if "J" in words: mv["J"]=words["J"]
        moves.append(mv)
    return moves

def filter_center_scale(moves, reach_bounds):
    REACH_X_MIN, REACH_X_MAX, REACH_Y_MIN, REACH_Y_MAX = reach_bounds
    reach_cx = (REACH_X_MIN + REACH_X_MAX) / 2
    reach_cy = (REACH_Y_MIN + REACH_Y_MAX) / 2

    first=next((k for k,m in enumerate(moves) if m["dE"]>1e-9 and m["Z"]<=FIRST_LAYER_Z),0)
    if first>0: first-=1
    moves=moves[first:]
    ex=[m["X"] for m in moves if m["dE"]>1e-9]; ey=[m["Y"] for m in moves if m["dE"]>1e-9]
    if not ex: ex=[m["X"] for m in moves]; ey=[m["Y"] for m in moves]
    cx,cy=(min(ex)+max(ex))/2,(min(ey)+max(ey))/2
    span=max(max(ex)-min(ex),max(ey)-min(ey))
    s=1.0
    print(f"part span {span:.1f}mm, center ({cx:.1f},{cy:.1f}), scale {s:.3f}")
    print(f"reach bounds X[{REACH_X_MIN:.1f},{REACH_X_MAX:.1f}] Y[{REACH_Y_MIN:.1f},{REACH_Y_MAX:.1f}] mm, "
          f"target center ({reach_cx:.1f},{reach_cy:.1f})")

    # first pass: center onto the reachable box, clamp
    centered=[]
    for m in moves:
        x,y=(m["X"]-cx+reach_cx),(m["Y"]-cy+reach_cy)
        x = max(REACH_X_MIN, min(REACH_X_MAX, x))
        y = max(REACH_Y_MIN, min(REACH_Y_MAX, y))
        m=dict(m); m["X"],m["Y"]=x,y; m["Z"]=max(m["Z"],Z_CLAMP)
        centered.append(m)

    # compute model bbox from extruding moves, percentile to exclude purge lines
    px=sorted([m["X"] for m in centered if m["dE"]>1e-9])
    py=sorted([m["Y"] for m in centered if m["dE"]>1e-9])
    if len(px)>10:
        lo=int(len(px)*0.05); hi=int(len(px)*0.95)
        model_x_min, model_x_max = px[lo], px[hi]
        lo=int(len(py)*0.05); hi=int(len(py)*0.95)
        model_y_min, model_y_max = py[lo], py[hi]
        pad_x = (model_x_max - model_x_min) * 0.2
        pad_y = (model_y_max - model_y_min) * 0.2
        print(f"model bbox (percentile): X[{model_x_min:.1f},{model_x_max:.1f}] Y[{model_y_min:.1f},{model_y_max:.1f}]")
    else:
        model_x_min, model_x_max = REACH_X_MIN, REACH_X_MAX
        model_y_min, model_y_max = REACH_Y_MIN, REACH_Y_MAX
        pad_x, pad_y = 0.0, 0.0

    # second pass: drop purge lines, demote degenerate arcs
    purged = 0
    out=[]
    for m in centered:
        if (m["X"] < model_x_min - pad_x or m["X"] > model_x_max + pad_x or
            m["Y"] < model_y_min - pad_y or m["Y"] > model_y_max + pad_y):
            purged += 1
            continue
        if m["cmd"] in ("G2","G3") and out:
            sx,sy=out[-1]["X"],out[-1]["Y"]
            ex,ey=m["X"],m["Y"]
            i,j=m.get("I",0.0),m.get("J",0.0)
            ccx,ccy=sx+i,sy+j
            r=math.hypot(i,j)
            a0=math.atan2(sy-ccy,sx-ccx)
            a1=math.atan2(ey-ccy,ex-ccx)
            span_ang=(a0-a1) if m["cmd"]=="G2" else (a1-a0)
            if span_ang<=0: span_ang+=2*math.pi
            sign=-1 if m["cmd"]=="G2" else 1
            amid=a0+sign*span_ang/2
            mx,my=ccx+r*math.cos(amid),ccy+r*math.sin(amid)
            cross_norm=abs((mx-sx)*(ey-sy)-(my-sy)*(ex-sx))
            if cross_norm<CROSS_NORM_MIN:
                m["cmd"]="G1"
                m.pop("I",None); m.pop("J",None)
        out.append(m)
    if purged: print(f"removed {purged} moves outside model bbox (purge/cleaning lines)")
    return out

def split_arcs(moves):
    out=[]; px,py=0.0,0.0
    for m in moves:
        if m["cmd"] not in ("G2","G3") or ("I" not in m and "J" not in m):
            out.append(m); px,py=m["X"],m["Y"]; continue
        sx,sy=px,py; ex,ey=m["X"],m["Y"]
        ccx,ccy=sx+m.get("I",0.0),sy+m.get("J",0.0)
        r=math.hypot(sx-ccx,sy-ccy)
        a0=math.atan2(sy-ccy,sx-ccx); a1=math.atan2(ey-ccy,ex-ccx)
        span=(a0-a1) if m["cmd"]=="G2" else (a1-a0)
        sign=-1 if m["cmd"]=="G2" else 1
        if span<=0: span+=2*math.pi
        if span<=math.pi+1e-6:
            out.append(m); px,py=ex,ey; continue
        amid=a0+sign*span/2
        mx,my=ccx+r*math.cos(amid),ccy+r*math.sin(amid)
        h1=dict(m); h1["X"],h1["Y"]=mx,my; h1["I"],h1["J"]=ccx-sx,ccy-sy
        h2=dict(m); h2["I"],h2["J"]=ccx-mx,ccy-my
        if "E" in m and m["dE"]>0: h1["E"]=m["E"]-m["dE"]/2
        out.extend([h1,h2]); px,py=ex,ey
    return out

def rdp(points,tol):
    if len(points)<3: return points
    a,b=points[0],points[-1]
    abx,aby=b[0]-a[0],b[1]-a[1]; L=math.hypot(abx,aby)
    dmax,idx=0.0,0
    for i in range(1,len(points)-1):
        p=points[i]
        d=math.hypot(p[0]-a[0],p[1]-a[1]) if L<1e-12 else abs(abx*(a[1]-p[1])-(a[0]-p[0])*aby)/L
        if d>dmax: dmax,idx=d,i
    if dmax>tol: return rdp(points[:idx+1],tol)[:-1]+rdp(points[idx:],tol)
    return [a,b]

def simplify_and_dedup(moves):
    out=[]; run=[]
    def same_kind(a,b):
        return (abs(a["Z"]-b["Z"])<1e-9 and (a["dE"]>1e-9)==(b["dE"]>1e-9)
                and a["cmd"]=="G1" and b["cmd"]=="G1")
    def flush():
        nonlocal run
        if not run: return
        if SIMPLIFY_TOL>0 and len(run)>2:
            pts=[(m["X"],m["Y"]) for m in run]
            keep=set(rdp(pts,SIMPLIFY_TOL))
            run=[m for m,p in zip(run,pts) if p in keep]
        out.extend(run); run=[]
    for m in moves:
        if m["cmd"]!="G1":
            flush(); out.append(m); continue
        if run and not same_kind(run[-1],m): flush()
        run.append(m)
    flush()
    final=[]
    for m in out:
        if m["cmd"] in ("G2","G3"):
            final.append(m); continue
        if final and all(abs(final[-1][k]-m[k])<1e-6 for k in ("X","Y","Z")): continue
        final.append(m)
    return final

def demote_flat_arcs(moves, min_sweep_deg=0.5):
    out=[]; px,py=0.0,0.0
    for m in moves:
        if m["cmd"] in ("G2","G3") and ("I" in m or "J" in m):
            ccx,ccy=px+m.get("I",0.0),py+m.get("J",0.0)
            r=math.hypot(px-ccx,py-ccy)
            chord=math.hypot(m["X"]-px,m["Y"]-py)
            if r>1e-9 and chord/(2*r)<=1.0:
                sweep=2*math.asin(chord/(2*r))
                if math.degrees(sweep)<min_sweep_deg:
                    m=dict(m); m["cmd"]="G1"; m.pop("I",None); m.pop("J",None)
        out.append(m); px,py=m["X"],m["Y"]
    return out

def select_layers(moves,n_layers):
    if not n_layers: return moves
    zs=sorted(set(m["Z"] for m in moves))
    if n_layers>=len(zs): return moves
    idx=[round(i*(len(zs)-1)/(n_layers-1)) for i in range(n_layers)] if n_layers>1 else [0]
    keep=set(zs[i] for i in idx)
    out=[]; prev_z=None
    for m in moves:
        if m["Z"] not in keep: continue
        if m["Z"]!=prev_z:
            m=dict(m); m["dE"]=0.0
        out.append(m); prev_z=m["Z"]
    print(f"layer select: kept {len(keep)} of {len(zs)} layers (Z {min(keep):.1f} to {max(keep):.1f})")
    return out

def fmt(moves):
    lines=[]
    for m in moves:
        parts=[m["cmd"],f"X{m['X']:.4f}",f"Y{m['Y']:.4f}",f"Z{m['Z']:.4f}"]
        if "I" in m: parts.append(f"I{m['I']:.4f}")
        if "J" in m: parts.append(f"J{m['J']:.4f}")
        if abs(m["E"])>1e-9 and abs(m["dE"])>1e-9: parts.append(f"E{m['E']:.4f}")
        if m["F"]>0: parts.append(f"F{m['F']:.0f}")
        lines.append(" ".join(parts))
    return lines

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output",nargs="?",default=None)
    ap.add_argument("-l","--layers",type=int,default=0,
                    help="number of evenly spaced layers to keep across full height, 0 = all (default)")
    ap.add_argument("--reach-csv",default="reachable_points.csv",
                    help="CSV from the reachability node used to compute workspace clamp bounds")
    args=ap.parse_args()
    outfile=args.output or args.input.rsplit(".",1)[0]+"_parsed.txt"

    try:
        reach_bounds=load_reach_bounds(args.reach_csv)
    except (FileNotFoundError, ValueError, KeyError) as e:
        sys.exit(f"error: could not load reach bounds from {args.reach_csv} ({e}); "
                 f"run the reachability node first or pass --reach-csv <path>")

    moves=parse_moves(load_lines(args.input))
    print(f"parsed {len(moves)} raw moves")
    moves=split_arcs(moves)
    moves=filter_center_scale(moves, reach_bounds)
    moves=simplify_and_dedup(moves)
    moves=demote_flat_arcs(moves)
    moves=select_layers(moves,args.layers)
    open(outfile,"w").write("\n".join(fmt(moves)))
    print(f"{len(moves)} moves -> {outfile}")