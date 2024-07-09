from math import pi
from pygame.math import Vector2 as V2
import pygame
import pygame.gfxdraw as gfx
import thorpy as tp

class Segment:

    def __init__(self, a:V2, b:V2):
        self.a = a
        self.b = b
        self.delta = b-a
        self.unit = self.delta.normalize()
        self.ar:V2 = V2()
        self.br:V2 = V2()

    def refresh(self):
        self.delta = self.b-self.a
        self.unit = self.delta.normalize()

class Turn:
    def __init__(self, s1:Segment, s2:Segment):
        self.s1 = s1
        self.s2 = s2
        self.R:float = 1.
        self.c:V2 = V2()
        self.d = -1
        self.angle_i = 0
        self.angle_f = 0
        self.magic = 0

    def q(self):
        return self.s1.b

    # def bissec(self, length:float) -> V2:
    #     return self.q() + (self.s2.unit/2 - self.s1.unit/2)*length

    def bissec(self, length:float) -> V2:
        bissec_dir_unit = (self.s2.unit - self.s1.unit)/2
        if bissec_dir_unit.length() == 0:
            return self.q() + self.s2.unit.rotate(90)*length
        return self.q() + bissec_dir_unit.normalize()*length

    # def bissec(self, length:float) -> V2:
    #     angle = self.s2.unit.angle_to(self.s1.unit)
    #     return self.q() + self.s2.unit.rotate(-angle/2)*length

    def compute_c(self, point_on_bissect:V2):
        self.c = V2(point_on_bissect)
        v1 = point_on_bissect - self.q()
        v2 = -self.s1.unit
        br = v1.project(v2)
        self.s1.br = self.q() + br
        self.s2.ar = self.q() + self.s2.unit * br.length()
        self.R = (self.c - self.s1.br).length()
        self.d = self.c.distance_to(self.q())
        #
        self.angle_i = V2(1,0).angle_to(self.s1.br-self.c)
        self.angle_f = V2(1,0).angle_to(self.s2.ar-self.c)
        self.clockwise = self.s1.unit.cross(self.s2.unit) > 0
        if not self.clockwise:
            self.angle_i, self.angle_f = self.angle_f, self.angle_i

        

DEFAULT_D:float = 120
class Loop:

    def __init__(self, points_list:list[V2], centers_list:list[V2|None]):
        self.segments:list[Segment] = []
        self.turns:list[Turn] = []
        #
        points = [V2(*p) for p in points_list]
        for i in range(len(points)):
            j = (i+1) % len(points)
            self.segments.append(Segment(points[i], points[j]))
        for i in range(len(self.segments)):
            j = (i+1) % len(self.segments)
            turn = Turn(self.segments[i], self.segments[j])
            i_track = self.i_loop_to_track(i)
            value = centers_list[i_track]
            if value is None:
                turn.compute_c(turn.bissec(DEFAULT_D))
            else:
                turn.compute_c(turn.bissec(value))
            self.turns.append(turn)

    def i_track_to_loop(self, i):
        return (i-1)%len(self.segments)
    
    def i_loop_to_track(self, i):
        return (i+1)%len(self.segments)

    def check_sanity(self):
        for i,s in enumerate(self.segments):
            j = (i+1) % len(self.segments)
            # if self.segments[j].a != s.b:
            if not self.segments[j].a is s.b:
                print("Segments not connected (not same address)", i, j)
                return False
        return True

    def get_indices_of_turn_number(self, i_turn:int) -> tuple[int,int]:
        point_b_segment = (i_turn-1)%len(self.turns)
        point_a_segment = i_turn
        return point_b_segment, point_a_segment

    def draw(self, surface):
        # for i,s in enumerate(self.segments):
        #     j = (i-1) % len(self.segments)
        #     s2 = self.segments[j]
        #     # pygame.draw.line(surface, (220,)*3, s.a, s.b)
        #     pygame.draw.line(surface, (220,)*3, s.ar, s.br)
        #     # pygame.draw.rect(surface, (0,0,155), pygame.Rect(s.ar, (6,6)))
        #     # br = tp.Text("ar"+str(i))
        #     # br.set_center(*s.ar)
        #     # br.draw()
        #     # pygame.draw.rect(surface, (0,155,0), pygame.Rect(s.br, (6,6)))
        #     # br = tp.Text("br"+str(i))
        #     # br.set_center(*s.br)
        #     # br.draw()
        for i,t in enumerate(self.turns):
            #les angles sont mesurés dans le sens horaire ! (wtf?)
            gfx.circle(surface, round(t.c.x), round(t.c.y), 6, (220,)*3)
            # gfx.arc(surface, round(t.c.x), round(t.c.y), round(t.R), round(t.angle_i), round(t.angle_f), (0,)*3)
            # pygame.draw.line(surface, (0,0,0), t.q(), t.bissec(100), 4) #BISSEC
            # br = tp.Text(str(i)+"--->"+str(t.c))
            br = tp.Text(str(i)+"-->"+str(self.i_loop_to_track(i)))
            # br = tp.Text(str(i) + " " + str(round(t.angle_f-t.angle_i)), 30, (0,0,255))
            # br = tp.Text(str(i) + " " + str(round(t.R)), 30, (0,0,255))
            # br = tp.Text(str(round(t.angle_i))+"  ;  "+ str(round(t.angle_f)), 30, (0,0,255))
            # br = tp.Text(str(t.clockwise))
            # if t.clockwise:
            #     br.set_font_color((0,0,255))
            br.set_center(*t.c)
            br.draw()

    def to_points(self, d_step:float) -> tuple[list[V2], list[int]]:
        def write_circle_points(t:Turn):
            points = []
            delta_angle = t.angle_f - t.angle_i
            if t.angle_i > 0 and t.angle_f < 0:
                delta_angle = 360 - abs(delta_angle)
            length = t.R * 2 * pi * abs(delta_angle)/360
            n_points = int(length/d_step)
            for i in range(n_points): #+1 ?
                angle = t.angle_i + i*delta_angle/n_points
                point = t.c + V2(1,0).rotate(angle)*t.R
                points.append(point)
            if not t.clockwise:
                points.reverse()
            # return [t.s1.br] + points + [t.s2.ar]
            return points
        def write_straight_points(t:Turn):
            points = []
            delta = t.s1.br - t.s1.ar
            step = delta.normalize() * d_step
            n_steps = int(delta.length() / step.length()) + 1
            for i in range(n_steps):
                points.append(t.s1.ar + i*step)
            return points
        ################
        points = []
        i_turn = 0
        points_to_ref = []
        for t in self.turns:
            new_points = []
            new_points += write_straight_points(t)
            new_points += write_circle_points(t)
            i_turn += 1
            # i = self.i_loop_to_track(i_turn)
            points += new_points
            # half = len(new_points)//2
            # points_to_ref += [i_turn]*half + [(i_turn+1)%len(self.turns)]*half
            points_to_ref += [i_turn]*len(new_points)
        return points, points_to_ref
        # newp, newr = [], []
        # TOL = d_step*0.5
        # for i in range(len(points)):
        #     j = (i+1) % len(points)
        #     p = points[i]
        #     q = points[j]
        #     if p.distance_to(q) > TOL:
        #         newp.append(p)
        #         newr.append(points_to_ref[i])
        # return newp, newr

    # def to_points(self) -> list[V2]:
    #     from math import pi
    #     points = []
    #     for t in self.turns:
    #         pos = V2(t.s1.br)
    #         angle = t.s1.unit.angle_to(t.s2.unit)
    #         n_points = int(abs(angle)) + 1
    #         delta_angle = angle/n_points
    #         length = t.R * 2 * pi * abs(delta_angle)/360
    #         # length = 1
    #         direction = t.s1.unit * length
    #         for i in range(n_points):
    #             # pos = pos + direction.rotate(delta_angle)
    #             direction.rotate_ip(delta_angle)
    #             pos += direction
    #             points.append(pos)
    #     return points
