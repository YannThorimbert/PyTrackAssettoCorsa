import pygame
from pygame import gfxdraw as gfx
import thorpy as tp
from pygame.math import Vector2 as V2

import numpy as np
from scipy.interpolate import splprep, splev
import pickle
import tkinter as tk
from tkinter import filedialog

pygame.init()

W,H = 2700, 1200
screen = pygame.display.set_mode((W, H))
tp.init(screen)

MAGNET1 = 20
MAGNET2 = 20


def tri(v,z=0):
    return (v[0], v[1], z)

def make_land_triangles(land_points, lane_points, profile, flip_faces=False):
    assert len(profile) == len(lane_points)
    triangles = []
    last_land_point = None
    for i in range(len(lane_points)+1):
        alt_q = profile[i%len(lane_points)][1]
        q = lane_points[i%len(lane_points)]
        alt_r = profile[(i+1)%len(lane_points)][1]
        r = lane_points[(i+1)%len(lane_points)]
        _, s = track.closest_point(q, float("inf"), candidates=land_points)
        # distances = [p.distance_to(q) for p in land_points]
        if flip_faces:
            triangles.append((tri(q,alt_q), tri(s), tri(r,alt_r)))
        else:
            triangles.append((tri(q,alt_q), tri(r,alt_r), tri(s)))
        if last_land_point and s != last_land_point:
            if flip_faces:
                triangles.append((tri(q,alt_q), tri(last_land_point), tri(s)))
            else:
                triangles.append((tri(q,alt_q), tri(s), tri(last_land_point)))
        last_land_point = s
    # if flip_faces:
    #     triangles.append((tri(q,alt_q), tri(last_land_point), tri(s)))
    # else:
    #     triangles.append((tri(q,alt_q), tri(s), tri(last_land_point)))
    return triangles

def write_triangles(triangles, name):
    with open(name+".stl", "w") as f:
        f.write(f"solid {name}\n")
        for i in range(len(triangles)):
            a,b,c = triangles[i]
            f.write(f"facet normal 0 0 0\nouter loop\n")
            f.write(f"vertex {a[0]} -{a[1]} {a[2]}\n")
            f.write(f"vertex {b[0]} -{b[1]} {b[2]}\n")
            f.write(f"vertex {c[0]} -{c[1]} {c[2]}\n")
            f.write("endloop\nendfacet\n")
        f.write(f"endsolid {name}\n")

def dist(p1, p2) -> float:
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

def dist_sqr(p1,p2) -> float:
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def get_spline(pts, close_loop, num:int) -> list[tuple[float,float]]:
    if close_loop:
        pts = pts + [pts[0]]
    x = [point[0] for point in pts]
    y = [point[1] for point in pts]
    tck, u = splprep([x, y], s=0, per=close_loop)
    u_new = np.linspace(0, 1, num=num)
    new_points = splev(u_new, tck)
    new_points_list = list(zip(new_points[0], new_points[1]))
    if close_loop:
        # new_points_list[-1] = new_points_list[0]
        new_points_list.pop()
    return new_points_list

def avg(pts):
    avg = []
    delta = len(pts)//int(e_avg_land.get_value())
    if delta < 1: delta = 1
    for i in range(0, len(pts)-delta, delta):
        x = sum([p[0] for p in pts[i:i+delta]])/delta
        y = sum([p[1] for p in pts[i:i+delta]])/delta
        avg.append((x,y))
    return avg


def smoothstep(t) -> float:
    return 3 * t**2 - 2 * t**3

def smooth_profile(profile, num:int) -> list[tuple[float,float]]:
    assert profile[-1][1] == profile[0][1]
    x = [point[0] for point in profile]
    y = [point[1] for point in profile]
    new_x = []
    new_y = []
    for i in range(num):
        t = i/num
        for j in range(len(x)-1):
            if x[j] <= t <= x[j+1]:
                tt = (t-x[j])/(x[j+1]-x[j])
                new_x.append(t)
                new_y.append(y[j] + (y[j+1]-y[j]) * smoothstep(tt))
                break
    # import matplotlib.pyplot as plt
    # plt.plot(new_x, new_y, "--")
    # plt.plot(x,y,"o")
    # plt.show()
    return list(zip(new_x, new_y))



class Track:

    def __init__(self):
        f = 0.5
        # self.profile_L:list[float] = [(0., 0.), (0.22, 0), (0.33, 70*f), (0.37, 100*f), (0.5, 20*f), (0.62, 0.),  (1., 0.)]
        # self.profile_R:list[float] = [(0., 0.), (0.22, 0), (0.33, 70*f), (0.37, 70*f), (0.5, 20*f), (0.62, 0.), (1., 0.)]
        self.profile_L:list[float] = [(0., 0.), (1., 0.)]
        self.profile_R:list[float] = [(0., 0.), (1., 0.)]
        self.points:list[tuple[float,float]]  = []
        self.left_lane:list[tuple[float,float]] = []
        self.right_lane:list[tuple[float,float]]  = []
        self.left_land:list[tuple[float,float]]  = []
        self.right_land:list[tuple[float,float]]  = []
        self.smoothed_profile_L = []
        self.smoothed_profile_R = []
        self.turns_L:list[int] = []
        self.turns_R:list[int] = []
        self.added_turns_L:list[int] = []
        self.added_turns_R:list[int] = []
        #
        self.close_loop = True
        self.modified_point = None
        self.need_refresh = True
        self.width = 16
        self.auto_kerbs = True

    def save_to_disk(self) -> None: 
        root = tk.Tk()
        root.withdraw()
        fn = filedialog.asksaveasfilename(title="Choose a project", filetypes=[("Track files", "*.track")])
        if fn:
            if not fn.endswith(".track"):
                fn += ".track"
            with open(fn, 'wb') as file:
                pickle.dump(self.points, file)


    def load_from_disk(self, fn=None):
        if not fn:
            root = tk.Tk()
            root.withdraw()
            fn = filedialog.askopenfilename(title="Choose a project", filetypes=[("Track files", "*.track")])
        with open(fn, 'rb') as file:
            self.points = pickle.load(file)
            self.mark_refresh()
            
    def insert_profile_point(self, x, profile):
        for i in range(len(profile)-1):
            if profile[i][0] <= x <= profile[i+1][0]:
                t = (x-profile[i][0])/(profile[i+1][0]-profile[i][0])
                new_y = profile[i][1] + t * (profile[i+1][1]-profile[i][1])
                profile.insert(i+1, (x, new_y))
                return

    def draw_polygon(self, pts:list[tuple], col, thick, fill=True) -> None:
        if len(pts) > 1 and thick > 0:
            if fill:
                gfx.filled_polygon(screen, pts, col)
            pygame.draw.aalines(screen, (0,0,0), self.close_loop, pts, thick)

    def refresh_if_needed(self, force=False, also_force_land=False):
        if self.need_refresh or force:
            self.width = e_width.get_value()
            num = int(e_display.get_value())
            if num >= 1 and len(self.points) >= 4:
                # self.left_lane = self.get_spline(num*len(self.points))
                self.left_lane = get_spline(self.points, self.close_loop, num*len(self.points))
                self.right_lane = self.build_lane(self.left_lane, self.width, 90)
                self.refresh_land_light()
                if also_force_land:
                    self.refresh_land()
                self.turns_L, self.turns_R = self.detect_kerbs()
                self.smoothed_profile_L = smooth_profile(self.profile_L, len(self.left_lane))
                self.smoothed_profile_R = smooth_profile(self.profile_R, len(self.right_lane))
                self.need_refresh = False

    def build_lane(self, pts, width, angle) -> list[tuple]:
        lane = []
        for i in range(len(pts)):
            idx_next = (i+1)%len(pts)
            idx_prev = (i-1)%len(pts)
            prev_point = V2(pts[idx_prev])
            here = V2(pts[i])
            next_point = V2(pts[idx_next])
            if here.distance_squared_to(next_point) < 1e-6:
                dir_prev = (here-prev_point).normalize()
                right_p = here + width * dir_prev.rotate(angle)
                lane.append((right_p.x, right_p.y))
                continue
            elif here.distance_squared_to(prev_point) < 1e-6:
                dir_next = (next_point-here).normalize()
                right_p = here + width * dir_next.rotate(angle)
                lane.append((right_p.x, right_p.y))
                continue
            else:
                if here.distance_squared_to(prev_point) < 1e-6 or here.distance_squared_to(next_point) < 1e-6:
                    lane.append(here)
                    continue
                dir_prev = (here-prev_point).normalize()
                dir_next = (next_point-here).normalize()
                avg = (dir_prev.rotate(angle) + dir_next.rotate(angle))/2
                right_p = here + width * avg
                lane.append((right_p.x, right_p.y))
        ######################
        tolerance = dist(track.left_lane[1], track.left_lane[0])*2
        # print("TOLERANCE", tolerance)
        # for i, coord in enumerate(lane):
        #     j = (i+1)%len(lane)
        #     d = dist(V2(lane[i]), V2(lane[j]))
        #     if d > tolerance:
        #         print("big distance", i, d)
        #         med = (V2(lane[i]) + V2(lane[j]))/2
        #         lane[i] = (med.x, med.y)
        #         lane[j] = (med.x, med.y)
        return lane


    def mark_refresh(self, *args):
        # print("marked refresh")
        self.need_refresh = True

    def closest_point(self, coord, threshold, candidates=None) -> tuple[int, tuple[float,float]]:
        if candidates is None:
            candidates = self.points
        if len(candidates) == 0:
            return -1, None
        best_i = -1
        best_d = float("inf")
        for i, point in enumerate(candidates):
            d = dist(point, coord)
            if d < best_d:
                best_i = i
                best_d = d
        if best_d < threshold:
            return best_i, candidates[best_i]
        return -1, None
    
    def write_stl(self, name, L:list[tuple], R:list[tuple]):
        # print(len(L), len(R))
        LENGTH = len(L)
        assert len(L) == len(R)
        profile_L = smooth_profile(self.profile_L, LENGTH)
        profile_R = smooth_profile(self.profile_R, LENGTH)
        with open(name+".stl", "w") as f:
            f.write(f"solid {name}\n")
            for i in range(LENGTH):
                # print(profile_L[i][1])
                ip = i%LENGTH
                jp = (i+1)%LENGTH
                q = tri(L[ip], profile_R[ip][1])
                r = tri(L[jp], profile_R[jp][1])
                s = tri(R[ip], profile_L[ip][1])
                t = tri(R[jp], profile_L[jp][1])
                # if i == len(L)-1:
                #     t = tri(R[0], profile_L[0][1])
                f.write(f"facet normal 0 0 0\nouter loop\n")
                f.write(f"vertex {q[0]} -{q[1]} {q[2]}\n")
                f.write(f"vertex {r[0]} -{r[1]} {r[2]}\n")
                f.write(f"vertex {s[0]} -{s[1]} {s[2]}\n")
                f.write("endloop\nendfacet\n")
                f.write(f"facet normal 0 0 0\nouter loop\n")
                f.write(f"vertex {s[0]} -{s[1]} {s[2]}\n")
                f.write(f"vertex {r[0]} -{r[1]} {r[2]}\n")
                f.write(f"vertex {t[0]} -{t[1]} {t[2]}\n")
                f.write("endloop\nendfacet\n")
            f.write(f"endsolid {name}\n")

    def write_all_stls(self):
        self.refresh_if_needed(force=True)
        self.write_stl("1ROAD1", self.right_lane, self.left_lane)
        #########################################
        profile_L = smooth_profile(self.profile_L, len(self.left_lane))
        # build_land_points()
        # pluto garder le truc traditionnel, mais enlever les points non valides
        triangles = make_land_triangles(self.left_land, self.left_lane, profile_L)
        write_triangles(triangles, "1GRASS0")
        #
        profile_R = smooth_profile(self.profile_R, len(self.right_lane))
        triangles = make_land_triangles(self.right_land, self.right_lane, profile_R, flip_faces=True)
        write_triangles(triangles, "1GRASS1")
        # self.write_stl("land_left", self.right_land, self.right_lane)
        # self.write_stl("land_right", self.left_lane, self.left_land)
        # self.write_stl("kerbs_right", self.left_lane, self.left_land)
        self.write_kerbs_stl("1KERB_L", -90)
        self.write_kerbs_stl("1KERB_R", 90)

    def write_kerbs_stl(self, name, angle):
        triangles = []
        w = self.width/10
        dz = self.width/200
        ddz = 0.9*dz
        # ddz = 0
        if "L" in name:
            lane = self.left_lane
            turns = self.turns_L + self.added_turns_L
            flip_faces = False
            profile = smooth_profile(self.profile_L, len(self.left_lane))
            profile2 = smooth_profile(self.profile_R, len(self.right_lane))
        else:
            lane = self.right_lane
            turns = self.turns_R + self.added_turns_R
            flip_faces = True
            profile = smooth_profile(self.profile_R, len(self.right_lane))
            profile2 = smooth_profile(self.profile_L, len(self.left_lane))
        #################
        for i in turns:
            alt_p = profile[i][1]
            p = V2(lane[i])
            j = (i+1)%len(lane)
            alt_q = profile[j][1]
            q = V2(lane[j]) 
            r = p + (q-p).normalize().rotate(angle) * w
            s = q + (q-p).normalize().rotate(angle) * w
            devers_r = (profile2[i][1] - profile[i][1]) / w
            alt_r = alt_p - devers_r * dz
            devers_s = (profile2[j][1] - profile[j][1]) / w
            alt_s = alt_q - devers_s * dz
            if flip_faces:
                triangles.append((tri(p,alt_p),tri(s,alt_s+dz),tri(q,alt_q+dz-ddz)))
                triangles.append((tri(r,alt_r),tri(s,alt_s+dz),tri(p,alt_p)))
                triangles.append((tri(q,alt_q+dz-ddz),tri(p,alt_p),tri(q,alt_q))) #vertical
            else:
                triangles.append((tri(p,alt_p),tri(q,alt_q+dz-ddz),tri(s,alt_s+dz)))
                triangles.append((tri(r,alt_r),tri(p,alt_p),tri(s,alt_s+dz)))
                triangles.append((tri(q,alt_q+dz-ddz),tri(q,alt_q),tri(p,alt_p))) #vertical
        if triangles:
            write_triangles(triangles, name)
        else:
            print("NO KERBS:", name)
            import os
            if os.path.exists(name+".stl"):
                os.remove(name+".stl")


    def detect_kerbs(self):
        turnsL, turnsR = [], []
        # last_turn = None
        for i in range(1,len(self.left_lane)-1): #skip first and last
            idx_next = (i+1)%len(self.left_lane)
            idx_prev = (i-1)%len(self.left_lane)
            p = V2(self.left_lane[idx_prev])
            h = V2(self.left_lane[i])
            n = V2(self.left_lane[idx_next])
            angle = (h-p).angle_to(n-h)
            # print(i, angle)
            m = 2
            M = 180 -m 
            if m < angle < M:
                turnsR.append(i)
                # last_turn = "R", i
            elif -M < angle < -m:
                turnsL.append(i)
                # last_turn = "L", i
            # elif last_turn: #kerbs of the beginning of a straight (corner exit)
            #     if last_turn[0] == "R":
            #         turnsL.append(i)
            #         turnsL.append(i+1)
            #     else:
            #         turnsR.append(i)
            #         turnsR.append(i+1)
            #     last_turn = None
        return turnsL, turnsR
    
    def get_land_points(self, land_points) -> list[tuple]:
        if e_avg_land.get_value() < 10:
            pts = land_points
        else:
            pts = avg(land_points)
        # return pts
        # print("***", len(pts), LAND_LANE_FACTOR * self.width)
        track_pts = self.left_lane + self.right_lane + self.points
        valid_pts = []
        parameter = self.width * 4 * e_thresh_land.get_value() / 100
        for p in pts:
            coord = min(track_pts, key=lambda q: dist_sqr(p, q))
            d = dist(p, coord)
            if d > parameter:#LAND_LANE_FACTOR * self.width:
                valid_pts.append(p)
        print(len(valid_pts))
        return valid_pts
        # 
        # repopulated = []
        # for i in range(len(decimated)-1):
        #     repopulated.append(decimated[i])
        #     next_point = decimated[i+1]
        #     p = V2(decimated[i])
        #     q = V2(next_point)
        #     delta = (q-p)/2
        #     repopulated.append(tuple(p+delta))
        # return repopulated
        # return get_spline(repopulated, True, 1*len(repopulated))
        
    def get_raw_or_spline_point(self, ref_point):
        i, coord = track.closest_point(ref_point, MAGNET1)
        if coord:
            return i, coord, "raw"
        else:
            i, coord = track.closest_point(ref_point, MAGNET2, candidates=self.left_lane)
            if coord:
                return i, coord, "spline"
        return -1, None, None
    
    def remontada_spline(self, spline_index:int) -> int:
        """Returns the index of the next raw point"""
        i = spline_index
        n = 200
        while i < len(self.left_lane)-1:
            p = V2(self.left_lane[i])
            q = V2(self.left_lane[i+1])
            direction_unit = (q-p)/n
            for step in range(n+1):
                to_check = p + direction_unit * step
                k, _ = self.closest_point(to_check, 3)
                if k >= 0:
                    return k
            i += 1
        # assert False
        return len(self.left_lane)-1
    
    def refresh_land(self):
        self.refresh_land_light()
        self.right_land = self.get_land_points(self.right_land)
        self.left_land = self.get_land_points(self.left_land)

    def refresh_land_light(self):
        f = e_width_land.get_value() / 100 * 10
        # print("light", f, f*self.width)
        self.right_land = self.build_lane(self.right_lane, f*self.width, 90)
        self.left_land = self.build_lane(self.left_lane, f*self.width, -90)
    
track = Track()

# def spawn_valid_land_points(res_w, res_h):
#     # if tp.get_current_loop().iteration%30 != 0:
#     #     return
#     valid_points = []
#     for x in np.linspace(0, W, res_w):
#         for y in np.linspace(0, H, res_h):
#             p = V2(x,y)
#             coord = min(track.left_lane+track.right_lane+track.points, key=lambda q: p.distance_squared_to(q))
#             d = p.distance_to(coord)
#             if track.width * 3 < d < track.width * 12:
#                 valid_points.append(tuple(p))
#                 # pygame.draw.circle(screen, (0,0,0), p, 3)
#     return valid_points

# def build_land_points():
#     valid_points = spawn_valid_land_points(30,20)
#     track.left_land = valid_points
#     track.right_land = valid_points


def simulate_land_points():
    import random
    for i in range(len(track.left_land)):
        p = V2(track.left_land[i])
        d,q = min([(dist(p, q), q) for q in track.left_lane])
        force = 100 / (d+1e-3)
        if d < 1e-6:
            p += V2(random.uniform(-1,1), random.uniform(-1,1))
        else:
            p += (p-q).normalize() * force
        track.left_land[i] = (p.x, p.y)
    for i in range(len(track.right_land)):
        p = V2(track.right_land[i])
        d,q = min([(dist(p, q), q) for q in track.right_land])
        force = 100 / (d+1e-3)
        if d < 1e-6:
            p += V2(random.uniform(-1,1), random.uniform(-1,1))
        else:
            p += (p-q).normalize() * force
        track.right_land[i] = (p.x, p.y)

def my_stuff(): #do what you want with the display like in any pygame code you write
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w] or keys[pygame.K_s]:
        if keys[pygame.K_w]:
            cand = track.left_lane
            turns = track.added_turns_L
        else:
            cand = track.right_lane
            turns = track.added_turns_R
        i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1, candidates=cand)
        if i >= 0:
            if pygame.key.get_mods() & pygame.KMOD_CTRL:
                if i in turns:
                    turns.remove(i)
            elif not i in turns:
                turns.append(i)
    i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1, candidates=track.left_lane)
    prof = track.smoothed_profile_L
    if i >= 0 and i < len(prof):
        e_alt.set_text(f"Alt = {round(prof[i][1])} ({round(100*i/len(prof))}%)")
        e_alt.set_bottomleft(50, H-e_alt.get_current_height()-10)
    #
    track.refresh_if_needed()
    # if not e_refresh_land.get_value():
    #     simulate_land_points()
    draw_all()

def draw_all():
    # print(len(track.left_land), len(track.right_land))
    ################# ASPHALT AND LAND ###########################
    screen.fill((255,255,255))
    fill_color = e_fill_visual.get_value()
    if e_show_land.get_value():
        track.draw_polygon(track.left_land, (0,200,0), 2, fill_color)
        fill_color_right_lane = (0,200,0)
        # for i_p, point in enumerate(track.get_land_points(track.left_land)):
        #     pygame.draw.circle(screen, (0,0,0), point, 6)
    else:
        fill_color_right_lane = (255,255,255)
    track.draw_polygon(track.left_lane, (200,200,200), 1, fill_color)
    track.draw_polygon(track.right_lane, fill_color_right_lane, 1, fill_color)
    if e_show_land.get_value():
        track.draw_polygon(track.right_land, (255,255,255), 2, fill_color)
        # for i_p, point in enumerate(track.right_land):
        #     pygame.draw.circle(screen, (0,0,255), point, 2)
    ################ POINTS ############################
    for i_p, point in enumerate(track.points):
        # if track.close_loop and i_p == len(track.points)-1:
        #     color = (255,100,0)
        # else:
        #     color = (0,0,0)
        pygame.draw.circle(screen, (0,0,0), point, 6)
    ###################### PROFILE ############################
    if len(track.smoothed_profile_L) > 4 and e_show_alt.get_value():
        n_ticks = 40
        profile = track.smoothed_profile_L
        lane = track.left_lane
        for i in range(0,len(profile), len(profile)//n_ticks):
            alt = profile[i][1]
            rect = pygame.Rect(lane[i], (4,6+alt))
            pygame.draw.rect(screen, (50,50,50), rect)
        i_prof = 0
        for x,alt in track.profile_L[0:-1]: #last is first so skip it
            idx = int(x * len(profile))
            coord = lane[idx]
            rect = pygame.Rect(coord, (10,6+alt))
            pygame.draw.rect(screen, (0,200,0), rect)
            pygame.draw.rect(screen, (0,0,0), rect, 1)
            if rect.collidepoint(pygame.mouse.get_pos()):
                # t = tp.Text("Alt = "+str(round(track.profile_L[i_prof][1],3)), 40, (0,100,0))
                # t.set_center(*rect.center)
                # t.draw()
                pygame.draw.rect(screen, (255,0,0), rect)
                if pygame.key.get_pressed()[pygame.K_e]:
                    track.mark_refresh()
                    alt_L, alt_R, deletion = choose_alt_i(i_prof)
                    if deletion:
                        print("deleted", i_prof)
                        return
                    else:
                        track.profile_L[i_prof] = (x, alt_L)
                        track.profile_R[i_prof] = (x, alt_R)
            i_prof += 1
        ##########################################################
        profile = track.smoothed_profile_R
        lane = track.right_lane
        for i in range(0,len(profile), len(profile)//n_ticks):
            alt = profile[i][1]
            rect = pygame.Rect(lane[i], (4,6+alt))
            pygame.draw.rect(screen, (50,50,50), rect)
        i_prof = 0
        for x,alt in track.profile_R[0:-1]: #last is first so skip it
            idx = int(x * len(profile))
            coord = lane[idx]
            rect = pygame.Rect(coord, (10,6+alt))
            pygame.draw.rect(screen, (255,255,0), rect)
            pygame.draw.rect(screen, (0,0,0), rect, 1)
            if rect.collidepoint(pygame.mouse.get_pos()):
                # t = tp.Text("Alt = "+str(round(track.profile_R[i_prof][1],3)), 40, (155,155,0))
                # t.set_center(*rect.center)
                # t.draw()
                pygame.draw.rect(screen, (255,0,0), rect)
                if pygame.key.get_pressed()[pygame.K_e]:
                    track.mark_refresh()
                    alt_L, alt_R, deletion = choose_alt_i(i_prof)
                    if deletion:
                        print("deleted", i_prof)
                        return
                    else:
                        track.profile_L[i_prof] = (x, alt_L)
                        track.profile_R[i_prof] = (x, alt_R)
            i_prof += 1
    ################ TURNS (KERBS) ############################
    if e_show_kerbs.get_value():
        for i in track.turns_L + track.added_turns_L:
            pygame.draw.rect(screen, (255,0,0), pygame.Rect(track.left_lane[i], (6,6)))
        for i in track.turns_R + track.added_turns_R:
            pygame.draw.rect(screen, (0,0,255),  pygame.Rect(track.right_lane[i], (6,6)))
    #### HANDLE CURSOR ################################################
    i, coord, kind = track.get_raw_or_spline_point(pygame.mouse.get_pos())
    if kind == "raw":
        pygame.draw.circle(screen, (0,0,255), coord, 8)
        pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
    elif kind == "spline":
        pygame.draw.circle(screen, (0,0,255), coord, 5)
        pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_CROSSHAIR)
    else:
        pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
    # spawn_valid_land_points(30,20)
    e_alt.draw()


# def choose_alt(alt) -> int:
#     e = tp.SliderWithText("Altitude", 0, 1000, alt, 1000)
#     e.center_on(screen)
#     screenshot = screen.copy()
#     r = pygame.Surface((W,H))
#     r.fill((255,255,255))
#     r.set_alpha(190)
#     screenshot.blit(r, (0,0))
#     screen.fill((255,255,255))
#     e.launch_alone(lambda:screen.blit(screenshot, (0,0)))
#     return e.get_value()

def choose_alt_i(i) -> int:
    alt_L = track.profile_L[i][1]
    alt_R = track.profile_R[i][1]
    same = alt_L == alt_R
    a = tp.Labelled("Keep same altitude",tp.Checkbox(same))
    b = tp.SliderWithText("Altitude L", 0, 1000, alt_L, 1000, round_decimals=0)
    c = tp.SliderWithText("Altitude R", 0, 1000, alt_R, 1000, round_decimals=0)
    elements = [a,b,c]
    def delete_point():
        global deleted_point
        print("deleted", i, track.profile_L[i])
        track.profile_L.pop(i)
        track.profile_R.pop(i)
        tp.quit_current_loop()
        DeletedPoint.value = True
        track.refresh_if_needed(force=True)
        print(track.profile_L)
    if i > 0 and i < len(track.profile_L)-1:
        d = tp.Button("Delete altitude marker")
        # d.at_unclick = lambda: track.profile_L.pop(i) and track.profile_R.pop(i) and tp.quit_current_loop()
        d.at_unclick = delete_point
        elements.append(d)
    box = tp.TitleBox("Choose altitudes", elements)
    box.center_on(screen)
    #
    screenshot = screen.copy()
    r = pygame.Surface((W,H))
    r.fill((255,255,255))
    r.set_alpha(190)
    screenshot.blit(r, (0,0))
    screen.fill((255,255,255))
    #
    def refresh_b(*args):
        if a.get_value():
            c.set_value(b.get_value())
    def refresh_c(*args):
        if a.get_value():
            b.set_value(c.get_value())
    b.slider.dragger.at_drag = refresh_b
    b.slider.dragger.at_unhover = refresh_b
    c.slider.dragger.at_drag = refresh_c
    c.slider.dragger.at_unhover = refresh_c
    class DeletedPoint:
        value = False
    box.launch_alone(lambda:screen.blit(screenshot, (0,0)))
    # print("exiting", deleted_point)
    return b.get_value(), c.get_value(), DeletedPoint.value

# e_display = tp.TogglablesPool("Display", ["Raw points", "Splines 2", "Splines 4", "Splines 16"], 0)
e_display = tp.SliderWithText("Mesh resolution", 0, 512, 32, 100)
e_display.slider.dragger.at_drag = track.mark_refresh

e_width = tp.SliderWithText("Track width", 1, 64, 16, 100)
e_width.slider.dragger.at_drag = track.mark_refresh

e_avg_land = tp.SliderWithText("Avg Land", 0, 200, 70, 200)
e_avg_land.slider.dragger.at_drag = track.mark_refresh

e_thresh_land = tp.SliderWithText("Land Parameter", 0, 100, 51, 100)

e_width_land = tp.SliderWithText("Land Width", 0, 100, 30, 100)
e_width_land.slider.dragger.at_drag = track.mark_refresh

e_fill_visual = tp.Labelled("Fill color", tp.Checkbox(True))
e_show_kerbs = tp.Labelled("Kerbs", tp.Checkbox(False))
e_show_alt = tp.Labelled("Altitude", tp.Checkbox(False))
e_show_land = tp.Labelled("Grass", tp.Checkbox(True))

e_savep = tp.Button("Save project")
e_savep.at_unclick = track.save_to_disk

e_loadp = tp.Button("Load project")
e_loadp.at_unclick = track.load_from_disk

e_alt = tp.Text("Alt = 0", 30, (50,50,50))
e_alt.set_bottomleft(50, H-e_alt.get_current_height()-10)

e_refresh_land = tp.Button("Refresh Land")
e_refresh_land.at_unclick = track.refresh_land


e_save = tp.Button("Save STL")
e_save.at_unclick = track.write_all_stls

e_box_track = tp.Box([e_display, e_width])
e_box_land = tp.Box([e_avg_land, e_thresh_land, e_width_land, e_refresh_land])
e_box_draw = tp.Box([e_fill_visual, e_show_land, e_show_kerbs, e_show_alt])
e_box_disk = tp.Box([e_savep, e_loadp, e_save])



e_group = tp.Box([e_box_track, e_box_land, e_box_draw, e_box_disk], sort_immediately=False)
e_group.sort_children("v")
updater = e_group.get_updater()
e_group.stick_to("screen", "left", "left", (5,0))

track.refresh_if_needed(force=True)
track.load_from_disk("simple.track")
track.refresh_if_needed(force=True)

track.smoothed_profile_L = smooth_profile(track.profile_L, len(track.left_lane))
track.smoothed_profile_R = smooth_profile(track.profile_R, len(track.right_lane))

zoom = 1.

clock = pygame.time.Clock()
playing = True
while playing:
    clock.tick(40)
    events = pygame.event.get()
    mouse_rel = pygame.mouse.get_rel()
    for e in events:
        if e.type == pygame.QUIT:
            playing = False
            print("Stop playing")

        if e_group.get_rect_with_children().collidepoint(pygame.mouse.get_pos()):
            continue
        if e.type == pygame.MOUSEMOTION:
            if pygame.mouse.get_pressed()[0]:
                track.mark_refresh()
                if track.modified_point is not None:
                    track.points[track.modified_point] = e.pos
                else:
                    i, _ = track.closest_point(e.pos, MAGNET1)
                    if i >= 0:
                        track.points[i] = e.pos
                        track.modified_point = i
                    else:
                        track.modified_point = None
            else:
                track.modified_point = None
                if pygame.mouse.get_pressed()[1]: #shift track
                    track.mark_refresh()
                    for i in range(len(track.points)):
                        track.points[i] = (track.points[i][0]+mouse_rel[0], track.points[i][1]+mouse_rel[1])
        elif e.type == pygame.MOUSEBUTTONDOWN:
            if e.button == 3:
                track.mark_refresh()
                i, coord, kind = track.get_raw_or_spline_point(e.pos)
                if kind == "raw":
                    track.points.append(e.pos)
                elif kind == "spline":
                    next_i = track.remontada_spline(i)
                    if next_i >= 0:
                        print("Insert point at index", next_i, coord)
                        track.points.insert(next_i, coord)
                else:
                    print("add moar")
                    track.points.append(e.pos)
            elif e.button == 4 or e.button == 5:  # Mouse wheel up
                if e.button == 4:
                    zoom = 1.1
                else:
                    zoom = 1 / 1.1
                for i in range(len(track.points)):
                    track.points[i] = (track.points[i][0]*zoom, track.points[i][1]*zoom)
                track.width *= zoom
                e_width.set_value(track.width)
                track.width = e_width.get_value()
                track.mark_refresh()
        elif e.type == pygame.KEYDOWN:
            track.mark_refresh()
            if e.key == pygame.K_SPACE:
                track.close_loop = not track.close_loop
            elif e.key == pygame.K_a or e.key == pygame.K_d:
                i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1)
                if i >= 0:
                    if e.key == pygame.K_a:
                        j = (i-1)%len(track.points)
                        insert_index = i
                    else:
                        j = (i+1)%len(track.points)
                        insert_index = (i+1)%len(track.points)
                    new_pos = V2(track.points[j]) + (V2(track.points[i]) - V2(track.points[j]))/2
                    # print(track.points[i], track.points[j], new_pos)
                    track.points.insert(insert_index, tuple(new_pos))
            elif e.key == pygame.K_x:
                i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1)
                if i >= 0:
                    track.points.pop(i)
                track.refresh_if_needed(force=True)
            elif e.key == pygame.K_t:
                i, _ = track.closest_point(pygame.mouse.get_pos(), float("inf"), candidates=track.left_lane)
                x_elevation = i / len(track.left_lane)
                track.insert_profile_point(x_elevation, track.profile_L)
                track.insert_profile_point(x_elevation, track.profile_R)
                
    my_stuff() #do your stuff with display
    #update Thorpy elements and draw them
    updater.update(events=events,
                   mouse_rel=mouse_rel) #if you dont give a mouse_rel, Thorpy will call pygame mouse_rel() !
    pygame.display.flip()
pygame.quit()

#TODO: save added turns. allow disable auto turns
#TODO: working zoom (pas toucher a track width, vraiment histoire de camera)
#TODO: lancé de particles