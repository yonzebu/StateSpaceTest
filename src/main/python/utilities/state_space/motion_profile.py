


class MotionProfile(object):

    def __init__(self, v_max, a_max, xi, xf, vi=0, vf=0):
        self.v_max = v_max
        self.a_max = a_max
        self.xi = xi
        self.xf = xf
        self.vi = vi
        self.vf = vf

        self.ta = (self.v_max - self.vi) / self.a_max
        self.xa = 0.5 * self.a_max * (self.ta**2) + self.vi * self.ta
        if vi == vf:
            self.td = self.ta
            self.xd = self.xa
        else:
            self.td = (self.v_max - self.vf) / self.a_max
            self.xd = 0.5 * -self.a_max * (self.td**2) +  self.v_max * self.td
        self.xc = (self.xf - self.xi) - (self.xd + self.xa)