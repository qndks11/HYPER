import cv2, numpy as np, struct, os

SRC="/mnt/user-data/uploads/1783261857886_image.png"
OUT="/home/claude/work/driving_course"
MESH=os.path.join(OUT,"meshes")
os.makedirs(MESH,exist_ok=True)

img=cv2.imread(SRC)
H,W=img.shape[:2]
MPP=0.10                      # 3 m / 30 px
CURB_H=0.12                   # curb / grass plateau height (m)
Xh=(W/2)*MPP; Yh=(H/2)*MPP    # half extents (m)  -> 37.0 x 50.4

# ---------- 1) clean texture: remove signs + road text ----------
mask=np.zeros((H,W),np.uint8)
boxes=[(8,252,60,304),(8,600,64,662),(676,798,740,866),      # three "20" signs
       (390,718,440,902),                                    # 평행주차 (vertical)
       (448,888,528,1000),                                   # 종료 종료
       (604,798,688,872),                                    # 출발 출발
       (534,712,602,898)]                                    # 대기실 text (keep orange box)
for x0,y0,x1,y1 in boxes:
    mask[max(0,y0):min(H,y1),max(0,x0):min(W,x1)]=255
mask=cv2.dilate(mask,np.ones((3,3),np.uint8))
course=cv2.inpaint(img,mask,6,cv2.INPAINT_TELEA)
cv2.imwrite(os.path.join(MESH,"course.png"),course)

# ---------- 2) green regions ----------
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
green=cv2.inRange(hsv,(35,60,30),(90,255,220))
k=np.ones((5,5),np.uint8)
green=cv2.morphologyEx(green,cv2.MORPH_CLOSE,k,iterations=3)
green=cv2.morphologyEx(green,cv2.MORPH_OPEN,k,iterations=1)
cnts,_=cv2.findContours(green,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
regions=[]
for c in cnts:
    if cv2.contourArea(c)<1800: continue
    peri=cv2.arcLength(c,True)
    ap=cv2.approxPolyDP(c,2.5,True).reshape(-1,2).astype(np.float64)
    if len(ap)<3: continue
    regions.append(ap)
print("green regions kept:",len(regions),"vert counts:",[len(r) for r in regions])

def px2world(pts):
    X=(pts[:,0]-W/2)*MPP
    Y=(H/2-pts[:,1])*MPP
    return np.stack([X,Y],1)

def signed_area(p):
    x=p[:,0]; y=p[:,1]
    return 0.5*np.sum(x*np.roll(y,-1)-np.roll(x,-1)*y)

def pt_in_tri(p,a,b,c,eps=-1e-9):
    d1=(p[0]-b[0])*(a[1]-b[1])-(a[0]-b[0])*(p[1]-b[1])
    d2=(p[0]-c[0])*(b[1]-c[1])-(b[0]-c[0])*(p[1]-c[1])
    d3=(p[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(p[1]-a[1])
    neg=(d1<eps)or(d2<eps)or(d3<eps); pos=(d1>-eps)or(d2>-eps)or(d3>-eps)
    return not(neg and pos)

def earclip(poly):
    # poly: Nx2 CCW (signed area>0)
    idx=list(range(len(poly))); tris=[]; guard=0
    while len(idx)>3 and guard<20*len(poly):
        guard+=1; n=len(idx); found=False
        for i in range(n):
            ia,ib,ic=idx[(i-1)%n],idx[i],idx[(i+1)%n]
            a,b,c=poly[ia],poly[ib],poly[ic]
            cross=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            if cross<=1e-9: continue          # reflex or collinear
            ok=True
            for j in idx:
                if j in (ia,ib,ic): continue
                if pt_in_tri(poly[j],a,b,c): ok=False;break
            if ok:
                tris.append((ia,ib,ic)); del idx[i]; found=True; break
        if not found:
            # fallback: force-clip most convex vertex to avoid stall
            best=None
            for i in range(len(idx)):
                ia,ib,ic=idx[(i-1)%len(idx)],idx[i],idx[(i+1)%len(idx)]
                a,b,c=poly[ia],poly[ib],poly[ic]
                cr=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
                if best is None or cr>best[0]: best=(cr,i,(ia,ib,ic))
            tris.append(best[2]); del idx[best[1]]
    if len(idx)==3: tris.append((idx[0],idx[1],idx[2]))
    return tris

facets=[]  # each: (n, v0,v1,v2)
def add_tri(v0,v1,v2):
    nvec=np.cross(v1-v0,v2-v0); ln=np.linalg.norm(nvec)
    nvec=nvec/ln if ln>1e-12 else np.array([0,0,1.0])
    facets.append((nvec,v0,v1,v2))

for poly in regions:
    w=px2world(poly)
    if signed_area(w)<0: w=w[::-1]           # make CCW
    tri=earclip(w)
    top=lambda P:np.array([P[0],P[1],CURB_H])
    bot=lambda P:np.array([P[0],P[1],0.0])
    # top (+z) and bottom (-z)
    for (i,j,k2) in tri:
        add_tri(top(w[i]),top(w[j]),top(w[k2]))
        add_tri(bot(w[i]),bot(w[k2]),bot(w[j]))
    # side walls
    n=len(w)
    for i in range(n):
        j=(i+1)%n
        Bi,Bj=bot(w[i]),bot(w[j]); Ti,Tj=top(w[i]),top(w[j])
        add_tri(Bi,Bj,Tj); add_tri(Bi,Tj,Ti)

print("total curb triangles:",len(facets))

# ---------- 3) write binary STL ----------
with open(os.path.join(MESH,"curbs.stl"),"wb") as f:
    f.write(b'\x00'*80); f.write(struct.pack('<I',len(facets)))
    for nvec,a,b,c in facets:
        f.write(struct.pack('<12fH',*nvec,*a,*b,*c,0))

# ---------- 4) ground quad OBJ + MTL ----------
with open(os.path.join(MESH,"ground.mtl"),"w") as f:
    f.write("newmtl course\nKa 1 1 1\nKd 1 1 1\nd 1\nillum 1\nmap_Kd course.png\n")
obj=f"""mtllib ground.mtl
v {-Xh} {Yh} 0
v {-Xh} {-Yh} 0
v {Xh} {-Yh} 0
v {Xh} {Yh} 0
vt 0 1
vt 0 0
vt 1 0
vt 1 1
vn 0 0 1
usemtl course
f 2/2/1 3/3/1 4/4/1
f 2/2/1 4/4/1 1/1/1
"""
open(os.path.join(MESH,"ground.obj"),"w").write(obj)

print("world size (m): %.1f x %.1f, curb height %.2f"%(2*Xh,2*Yh,CURB_H))
print("done assets")
