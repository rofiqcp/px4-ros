"""Virtual Ackermann->2 hub-motor differential projection; no steering servo."""
import math

def clamp(x,lo,hi):return max(lo,min(hi,x))

def drive(v,yaw_rate, wheelbase,track,radius,pole_pairs=15,gear=1.,
          max_speed=.35,max_steer=.45,max_erpm=1500):
    args=(v,yaw_rate,wheelbase,track,radius,pole_pairs,gear,max_speed,max_steer,max_erpm)
    if not all(math.isfinite(float(x)) for x in args):
        return 0.,0,0,0,0,'NONFINITE'
    if wheelbase<=0 or track<=0 or radius<=0 or pole_pairs<=0 or gear<=0:
        return 0.,0,0,0,0,'GEOMETRY_UNVERIFIED'
    if max_speed<=0 or max_erpm<=0 or max_steer<=0:
        return 0.,0,0,0,0,'LIMITS_INVALID'
    v=clamp(v,-max_speed,max_speed)
    # Ackermann model cannot rotate in place.
    if abs(v)<1e-4:return 0.,0,0,0,0,'STOP'
    steering=clamp(math.atan(wheelbase*yaw_rate/v),-max_steer,max_steer)
    yaw=v*math.tan(steering)/wheelbase
    left=v-yaw*track*.5
    right=v+yaw*track*.5
    scale=60.*pole_pairs*gear/(2.*math.pi*radius)
    erpm_left=int(round(left*scale))
    erpm_right=int(round(right*scale))
    peak=max(abs(erpm_left),abs(erpm_right))
    if peak>max_erpm:
        factor=max_erpm/peak
        left*=factor;right*=factor
        erpm_left=int(round(erpm_left*factor))
        erpm_right=int(round(erpm_right*factor))
    return steering,left,right,erpm_left,erpm_right,'PREVIEW_ONLY'
