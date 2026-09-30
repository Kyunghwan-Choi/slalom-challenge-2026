"""Versioned public observation/action helpers; SI state and acceleration input."""
import math
import json
from functools import lru_cache
from pathlib import Path

STATE_NAMES=('X_ref','Y_ref','psi','vx_COM','vy_COM','yaw_rate','mean_front_steer')

def gate_side_valid(y_ref,cone):
    """A forward REF crossing is valid only in the cone's designated half-plane."""
    return cone['pass_sign']*(y_ref-cone['y'])>0.

@lru_cache(maxsize=1)
def _pedal_parameters():
    data=json.loads(Path(__file__).with_name('model_parameters.json').read_text(encoding='utf-8'))
    return dict(zip(data['parameter_names'],data['parameters']))

def _number(command,name,limit):
    value=command[name]
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or abs(value)>limit+1e-12:
        raise ValueError(f'invalid {name}: expected finite number in [-{limit},{limit}]')
    return float(value)

def _applied_steering(requested,previous,cfg):
    rate=cfg['steering_change_per_step']
    return previous+max(-rate,min(rate,requested-previous))

def _longitudinal_terms(vx,cfg):
    p=_pedal_parameters();mu=cfg['friction'];vplus=max(vx,0.)
    drive_capacity=max(.1,mu*9.81*p['drive_mu_ratio'])
    brake_capacity=max(.1,mu*9.81*p['brake_mu_ratio'])
    drive_gain=max(.1,p['traction0']+p['traction_v']*vplus+p['traction_v2']*vplus*vplus,
                   p['traction_high_speed_floor'] if vplus>=p['high_speed_ramp_start'] else .1)
    h=math.tanh(vx/.2)
    ramp_fraction=max(0.,min(1.,(vplus-p['high_speed_ramp_start'])/
                             (p['high_speed_ramp_end']-p['high_speed_ramp_start'])))
    ramp=ramp_fraction*ramp_fraction*(3.-2.*ramp_fraction)
    coast=-p['drag0']*h-p['drag_v2']*vx*abs(vx)+p['high_speed_drag_relief']*ramp
    limit=cfg['pedal_limit']
    drive_max=drive_capacity*math.tanh(drive_gain*limit**p['throttle_exponent']/drive_capacity)
    brake_effect_max=brake_capacity*math.tanh(p['brake_gain']*limit**p['brake_exponent']/brake_capacity)*h
    return p,drive_capacity,brake_capacity,drive_gain,h,coast,drive_max,brake_effect_max

def pedal_acceleration(vx,signed_pedal,cfg):
    """Reduced-model a_x produced by a signed native pedal at the given speed."""
    if not math.isfinite(vx) or not math.isfinite(signed_pedal) or abs(signed_pedal)>cfg['pedal_limit']+1e-12:
        raise ValueError('invalid speed or signed pedal')
    p,dc,bc,gain,h,coast,_,_=_longitudinal_terms(vx,cfg)
    throttle=max(signed_pedal,0.);brake=max(-signed_pedal,0.)
    drive=dc*math.tanh(gain*throttle**p['throttle_exponent']/dc)
    braking=bc*math.tanh(p['brake_gain']*brake**p['brake_exponent']/bc)*h
    return coast+drive-braking

def acceleration_bounds(vx,cfg):
    """Model-achievable a_x interval under the published pedal limit."""
    if not math.isfinite(vx):raise ValueError('invalid speed')
    _,_,_,_,_,coast,drive_max,brake_effect_max=_longitudinal_terms(vx,cfg)
    if vx<0.:
        # While rolling backward, the fixed lower controller brakes to a stop.
        return coast-brake_effect_max,coast-brake_effect_max
    return coast-brake_effect_max,coast+drive_max

def _acceleration_to_pedal(vx,request,cfg):
    if vx<0.:
        # The fitted forward drive/brake inverse is nonmonotone in reverse.
        return -cfg['pedal_limit']
    p,dc,bc,gain,h,coast,drive_max,brake_effect_max=_longitudinal_terms(vx,cfg)
    lower,upper=acceleration_bounds(vx,cfg)
    target=max(lower,min(upper,request))
    if target>=coast:
        ratio=max(0.,min((target-coast)/dc,1.-1e-12))
        throttle=(dc*math.atanh(ratio)/gain)**(1./p['throttle_exponent'])
        return min(cfg['pedal_limit'],throttle)
    if h<=0.:return 0.
    magnitude=(coast-target)/h
    ratio=max(0.,min(magnitude/bc,1.-1e-12))
    brake=(bc*math.atanh(ratio)/p['brake_gain'])**(1./p['brake_exponent'])
    return -min(cfg['pedal_limit'],brake)

def apply_action(command,previous,cfg,vx):
    """Full mode: desired a_x -> open-loop native pedals, plus steering slew."""
    if not isinstance(command,dict) or set(command)!={'steering','acceleration'}:
        raise ValueError('act must return exactly steering and acceleration')
    requested=_number(command,'steering',cfg['steering_limit'])
    acceleration=_number(command,'acceleration',cfg['acceleration_request_limit_m_s2'])
    if not math.isfinite(vx):raise ValueError('invalid speed')
    steering=_applied_steering(requested,previous,cfg)
    pedal=_acceleration_to_pedal(vx,acceleration,cfg)
    return [steering,max(pedal,0.),max(-pedal,0.)]

def apply_easy_action(command,previous,cfg):
    """Easy mode's fixed lower tracker retains its native signed-pedal output."""
    if not isinstance(command,dict) or set(command)!={'steering','longitudinal'}:
        raise ValueError('easy tracker must return exactly steering and longitudinal')
    requested=_number(command,'steering',cfg['steering_limit'])
    pedal=_number(command,'longitudinal',cfg['pedal_limit'])
    steering=_applied_steering(requested,previous,cfg)
    return [steering,max(pedal,0.),max(-pedal,0.)]

def footprint_clearance(state,cone,cfg):
    x,y,psi=state[:3];c=math.cos(psi);s=math.sin(psi)
    xc=x-cfg['ref_to_com_m']*c;yc=y-cfg['ref_to_com_m']*s
    dx=cone['x']-xc;dy=cone['y']-yc
    lx=c*dx+s*dy;ly=-s*dx+c*dy
    return math.hypot(max(abs(lx)-cfg['footprint_length_m']/2,0.),max(abs(ly)-cfg['footprint_width_m']/2,0.))-cfg['cone_radius_m']

def inside_road(state,cfg):
    _,y,psi=state[:3]
    yc=y-cfg['ref_to_com_m']*math.sin(psi)
    extent=cfg['footprint_length_m']/2*abs(math.sin(psi))+cfg['footprint_width_m']/2*abs(math.cos(psi))
    return abs(yc)+extent<=cfg['road_half_width_m']

def scenario_from_config(cfg):
    result=dict(cfg)
    centers=cfg.get('cone_centers_m')
    if centers is None:
        centers=[[cfg['first_cone_x_m']+j*cfg['cone_spacing_m'],0.] for j in range(cfg['cone_count'])]
    if not isinstance(centers,list) or len(centers)!=cfg['cone_count']:
        raise ValueError('cone_centers_m must list exactly cone_count [x,y] pairs')
    cones=[]
    prior_x=float('-inf')
    for j,point in enumerate(centers):
        if (not isinstance(point,list) or len(point)!=2 or
            any(isinstance(q,bool) or not isinstance(q,(int,float)) or not math.isfinite(q) for q in point)):
            raise ValueError('each cone center must be a finite [x,y] pair')
        x,y=map(float,point)
        if x<=prior_x or x>=cfg['finish_x_m']:
            raise ValueError('cone X positions must increase and precede the finish')
        cones.append({'x':x,'y':y,'pass_sign':1 if j%2==0 else -1})
        prior_x=x
    result['cones']=cones
    return result
