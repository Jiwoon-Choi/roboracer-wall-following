"""Focused geometry and sensor validation checks, run in the ROS container."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
spec=importlib.util.spec_from_file_location('controller',str(Path(__file__).resolve().parents[1]/'autodrive_devkit/autodrive_roboracer/wall_follow_node.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def scan_for_wall(b=1.0,alpha=0.0):
    start=-3*math.pi/4;step=math.pi/720
    rays=[]
    for i in range(1080):
        a=start+i*step;denom=math.sin(a)-math.cos(a)*math.tan(alpha)
        rays.append(min(10.,b/denom) if denom>0 else 10.)
    return SimpleNamespace(ranges=rays,angle_min=start,angle_increment=step,range_min=.06,range_max=10.)

for angle in [-20,0,20]:
    s=scan_for_wall(alpha=math.radians(angle))
    left=m.WallFollowNode.get_wall_range(None,s,90.)
    diag=m.WallFollowNode.get_wall_range(None,s,45.)
    theta=math.pi/4
    estimate=math.atan2(diag*math.cos(theta)-left,diag*math.sin(theta))
    assert abs(estimate-math.radians(angle))<.005,(angle,estimate)

s=scan_for_wall();s.ranges=[10.]*1080
assert m.WallFollowNode.get_front_clearance(None,s)==10.
s.ranges[540]=.02
assert m.WallFollowNode.get_front_clearance(None,s)==0.
s.ranges=[float('nan')]*1080
assert m.WallFollowNode.get_front_clearance(None,s) is None
assert m.WallFollowNode.get_wall_range(None,s,90.) is None
s.ranges=[float('inf')]*1080
assert m.WallFollowNode.get_front_clearance(None,s)==10.
print('PASS: wall geometry signs, clear front, dangerously close front, invalid scan, infinite front')
