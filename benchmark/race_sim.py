#!/usr/bin/env python3
"""Offline simulation of a parallel race: Lazy Theta* and stock A* start together;
when stock finishes at t_s, wait up to w*t_s more for Lazy Theta*, take its path
if it is done, else stock's. Assumes two cores, no interference. Run from the
benchmark directory: python3 race_sim.py (reads the two results CSVs)."""
import csv, numpy as np, collections
Q=collections.defaultdict(dict)
for r in csv.DictReader(open('ahppa_multiworld_results.csv')):
    if r['planner'] in ('stock','ahppa_h'): Q[(r['world'],r['qid'])][r['planner']]=r
for r in csv.DictReader(open('ahppa_baselines_results.csv')):
    Q[(r['world'],r['qid'])][r['planner']]=r
ks=[k for k,v in Q.items() if v['stock']['ok']=='1']
g=lambda k,p,c: float(Q[k][p][c]) if Q[k][p][c] else np.nan
Ls=np.array([g(k,'stock','length') for k in ks]); Ts=np.array([g(k,'stock','ms') for k in ks])/1000; Cs=np.array([g(k,'stock','min_clear') for k in ks])
okl=np.array([Q[k]['lt_only']['ok']=='1' for k in ks])
Ll=np.where(okl,[g(k,'lt_only','length') for k in ks],np.inf); Tl=np.array([g(k,'lt_only','ms') for k in ks])/1000; Cl=np.array([g(k,'lt_only','min_clear') for k in ks])
def J(L,T,C,a,b): return (L/Ls + a*T + b*np.maximum(0,1-C)).mean()
print(f"{'a':>5} {'b':>4} | {'stock':>6} {'lt+fb':>6} | race w=0  0.5    1    2   | best")
for a in (0,0.01,0.03,0.1,0.3):
  for b in (0,0.3):
    fixed_s=J(Ls,Ts,Cs,a,b)
    T_fb=np.where(okl,Tl,Tl+Ts); fixed_l=J(np.where(okl,Ll,Ls),T_fb,np.where(okl,Cl,Cs),a,b)
    res=[]
    for w in (0,0.5,1,2):
        D=(1+w)*Ts; take=okl&(Tl<=D)
        L=np.where(take,Ll,Ls); T=np.where(take,Tl,np.maximum(Ts,np.minimum(Tl,D))); C=np.where(take,Cl,Cs)
        T=np.where(take,Tl,D)
        res.append(J(L,T,C,a,b))
    print(f"{a:5.2f} {b:4.1f} | {fixed_s:6.3f} {fixed_l:6.3f} | "+" ".join(f"{x:6.3f}" for x in res)+f" | {'race' if min(res)<min(fixed_s,fixed_l)-1e-4 else 'fixed'} by {min(fixed_s,fixed_l)-min(res):+.4f}")
for w in (0,0.5,1,2):
    D=(1+w)*Ts; take=okl&(Tl<=D); T=np.where(take,Tl,D)
    print(f"race w={w}: LT path used {take.mean()*100:.0f}%, length vs stock {100*(np.mean(np.where(take,Ll,Ls)/Ls)-1):+.2f}%, median time {np.median(T)*1000:.0f} ms, p95 {np.percentile(T,95)*1000:.0f} ms, total {T.sum():.0f}s  (stock: median {np.median(Ts)*1000:.0f}, p95 {np.percentile(Ts,95)*1000:.0f}, total {Ts.sum():.0f}s; lt alone total {np.where(okl,Tl,Tl+Ts).sum():.0f}s, p95 {np.percentile(np.where(okl,Tl,Tl+Ts),95)*1000:.0f})")
