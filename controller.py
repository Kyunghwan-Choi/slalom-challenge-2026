"""Untrained smoke-test baseline. Not a qualifying learning-based submission."""
import math

def clamp(x,lo,hi):return max(lo,min(hi,x))

class Controller:
    def reset(self,scenario,seed):
        self.scenario=scenario

    def act(self,observation):
        x,y,psi,vx,vy,r,delta=observation['state']
        cfg=self.scenario;look=4.;target_x=x+look
        # An internal path chosen by this baseline, not an assignment reference.
        origin=cfg['cones'][0]['x']-.5*cfg['cone_spacing_m']
        target_y=3.2*math.sin(math.pi*(target_x-origin)/cfg['cone_spacing_m']) if target_x>origin else 0.
        dx=look;dy=target_y-y
        lx=math.cos(psi)*dx+math.sin(psi)*dy
        ly=-math.sin(psi)*dx+math.cos(psi)*dy
        steer=clamp(math.atan2(2*2.776*ly,lx*lx+ly*ly)/.626671,-.8,.8)
        acceleration=clamp(1.5*(5.-vx),-3.,3.)
        return {'steering':steer,'acceleration':acceleration}
