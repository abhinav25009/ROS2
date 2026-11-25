import math, sys, os
sys.argv=['x','.','.']
exec(open('/Users/abhinavkumar/uss_projrct/ahppa_paper/world_maps.py').read().split("plt.rcParams.update")[0])
def look(cam, tgt):
    dx,dy,dz=[t-c for t,c in zip(tgt,cam)]
    yaw=math.atan2(dy,dx); pitch=math.atan2(-dz, math.hypot(dx,dy))
    cy,sy,cp,sp=math.cos(yaw/2),math.sin(yaw/2),math.cos(pitch/2),math.sin(pitch/2)
    return cam, (0.0,pitch,yaw), (cy*cp, -sy*sp, cy*sp, sy*cp)
def bounds(path):
    xs,ys=[],[]
    for k,n,(x,y,z,*_),d in shapes(path):
        if k=='box' and d[0]*d[1]>3000: continue
        r=(max(d[0],d[1])/2 if k=='box' else d[0])
        xs+= [x-r,x+r]; ys+=[y-r,y+r]
    return min(xs),max(xs),min(ys),max(ys)
V={}
for f in sorted(os.listdir('cw')):
    x0,x1,y0,y1=bounds('cw/'+f); cx,cy=(x0+x1)/2,(y0+y1)/2; L=max(x1-x0,y1-y0)
    V[f[:-4]]=('COL', [look((cx-0.40*L, cy-0.40*L, 0.50*L),(cx,cy,0))])
W='vm/worlds/'
V['house_3room']=('OUR',[look((-13,-11,10),(0,0,0)), look((-8.3,-5.3,2.6),(-3,1,0.6))])
V['house_2floor']=('OUR',[look((-14,-13,12),(2,0,1)), look((-6.5,-4.6,2.5),(3,0.5,1.0)), look((-6.3,4.4,5.8),(3,-1,2.5))])
V['glass_test']=('OUR',[look((-2.1,-1.9,2.5),(6,2.5,0.8)), look((10.5,6.5,2.6),(3,0,0.8))])
V['indoor_10x8x3']=('OUR',[look((-4.6,-3.6,2.7),(1,0.5,0.6))])
out=[]
for name,(kind,views) in V.items():
    src = f"$HOME/world-collection/worlds/{name}/{name}.sdf" if kind=='COL' else f"$HOME/Indoor-Drone-Navigation/worlds/{name}.sdf"
    vs=';'.join(f"{c[0]:.2f} {c[1]:.2f} {c[2]:.2f} {e[0]:.3f} {e[1]:.3f} {e[2]:.3f}|{q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {q[3]:.4f}" for c,e,q in views)
    out.append(f"{name}\t{src}\t{vs}")
open('views.tsv','w').write('\n'.join(out)+'\n'); print(open('views.tsv').read())
