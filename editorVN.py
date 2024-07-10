import os
import pygame
from pygame import gfxdraw as gfx
import thorpy as tp
from pygame.math import Vector2 as V2
from pygame.math import Vector3 as V3

import numpy as np
from scipy.interpolate import splprep, splev
import pickle
import tkinter as tk
from tkinter import filedialog

import radial as rad

pygame.init()

W,H = 2700, 1200
screen = pygame.display.set_mode((W, H))
tp.init(screen)

MAGNET1 = 20
MAGNET2 = 20
COLOR_POINT_HOVER = (0,255,0)



def tri(v,z=0):
    return (v[0], v[1], z)


def find_max_dist_on_lane(pts, profile_points) -> float:
    prof = smooth_profile(profile_points, len(pts))
    assert len(prof) == len(pts)
    distances = []
    for i in range(len(pts)):
        x,y = pts[i]
        z = prof[i][1]
        p = V3(x,y,z)
        j = (i+1)%len(pts)
        x2,y2 = pts[j]
        z2 = prof[j][1]
        q = V3(x2,y2,z2)
        d = p.distance_to(q)
        distances.append(d)
    return max(distances)

# def autoadapt_resolution():
#     max_dist = find_max_dist_on_lane(track.left_lane, track.profile_L)
#     print("max dist found", max_dist)
#     current_res = e_res.get_value()
#     new_res = int(current_res * max_dist)
#     e_res.set_value(new_res)
#     track.refresh_if_needed(force=True)


def make_land_triangles(land_points, lane_points, profile, flip_faces=False, cliffs=-1):
    assert len(profile) == len(lane_points)
    if cliffs < 0:
        #then we have to compute the height of land points in order to have a smooth surface
        new_land_pts = {}
        for i,p in enumerate(lane_points):
            q = min(land_points, key=lambda q: dist_sqr(p,q))
            #slow version
            if q in new_land_pts:
                new_land_pts[q].append(i)
            else:
                new_land_pts[q] = [i]
            # new_land_pts.setdefault(q, []).append(p) #faster ?
        avg_land_pts = []
        for q, indices in new_land_pts.items():
            tot_alt = 0
            tot_fac = 0
            for i in indices:
                d = dist(q, p)
                factor = 1 / d
                tot_fac += factor
                tot_alt += profile[i][1] * factor
            z = tot_alt/tot_fac
            avg_land_pts.append((q[0], q[1], z))
        land_points = avg_land_pts
    triangles = []
    previous_s = None
    for i in range(len(lane_points)+1):
        alt_q = profile[i%len(lane_points)][1]
        q = lane_points[i%len(lane_points)]
        alt_r = profile[(i+1)%len(lane_points)][1]
        r = lane_points[(i+1)%len(lane_points)]
        k, s = track.closest_point(q, float("inf"), candidates=land_points)
        # distances = [p.distance_to(q) for p in land_points]
        if cliffs < 0:
            alt_land = land_points[k][2]
        else:
            alt_land = cliffs #otherwise, user choose altitude of cliff bottom
        s = tri(s, alt_land)
        if flip_faces:
            triangles.append((tri(q,alt_q), s, tri(r,alt_r)))
        else:
            triangles.append((tri(q,alt_q), tri(r,alt_r), s))
        if previous_s and s != previous_s:
            if flip_faces:
                triangles.append((tri(q,alt_q), previous_s, tri(s,alt_land)))
            else:
                triangles.append((tri(q,alt_q), tri(s,alt_land), previous_s))
        previous_s = s
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

def add_devers(profile:list[tuple[float,float]], other:list[tuple[float,float]], length) -> list[tuple[float,float]]:
    assert len(profile) == len(other)
    new_profile = []
    for i in range(len(profile)):
        x1,z1 = profile[i]
        x2,z2 = other[i]
        devers = -(z2-z1)/track.width * length
        new_profile.append((x1, z1+devers))
    return new_profile

# def smooth_profile(profile, num:int) -> list[tuple[float,float]]:
#     profile[-1] = (1., profile[0][1]) #close loop
#     for i in range(len(profile)-1):
#         if profile[i][0] > profile[i+1][0]:
#             print(profile)
#             assert False, "profile not sorted"
#     x = [point[0] for point in profile]
#     y = [point[1] for point in profile]
#     new_x = []
#     new_y = []
#     for i in range(num):
#         fraction = i/num
#         for j in range(len(x)-1):
#             # k = (j+1)%len(x)
#             k = j + 1
#             if x[j] <= fraction < x[k]: #found the 2 x values that surround t
#                 fraction_in_segment = (fraction-x[j])/(x[k]-x[j])
#                 new_x.append(fraction)
#                 new_y.append(y[j] + (y[k]-y[j]) * smoothstep(fraction_in_segment))
#                 break
#     # import matplotlib.pyplot as plt
#     # plt.plot(new_x, new_y, "--")
#     # plt.plot(x,y,"o")
#     # plt.show()
#     return list(zip(new_x, new_y))

# Since the points are not necessarily evenly spaced, we interpolate according to distance, not index.
def smooth_profile(profile, num:int) -> list[tuple[float,float]]:
    profile[-1] = (1., profile[0][1]) #close loop
    for i in range(len(profile)-1):
        if profile[i][0] > profile[i+1][0]:
            print(profile)
            assert False, "profile not sorted"
    #1. [(x,y), ...] ---> [x, ...] and [y, ...]
    x = [point[0] for point in profile]
    y = [point[1] for point in profile]
    #2. Compute the total length of the profile
    tot_dist = 0
    i_to_dist = [0.]
    assert num == len(track.left_lane)
    for i in range(1, len(track.left_lane)):
        p = V2(track.left_lane[i])
        q = V2(track.left_lane[i-1])
        tot_dist += p.distance_to(q)
        i_to_dist.append(tot_dist)
    #3. Interpolate
    new_x = []
    new_y = []
    for i in range(num): #TODO: fusion with above loop
        fraction = i_to_dist[i]/tot_dist
        if fraction == 1:
            new_x.append(1.)
            new_y.append(y[-1])
            break
        for j in range(len(x)-1):
            # k = (j+1)%len(x)
            k = j + 1
            if x[j] <= fraction < x[k]: #found the 2 x values that surround t
                fraction_in_segment = (fraction-x[j])/(x[k]-x[j])
                new_x.append(fraction)
                new_y.append(y[j] + (y[k]-y[j]) * smoothstep(fraction_in_segment))
                break
        else:
            assert False, f"{fraction}, {i}, {tot_dist}, {x}, {j}->{k}"
    return list(zip(new_x, new_y))


class Track:

    def __init__(self):
        self.profile_L:list[float] = [(0., 0.), (1., 0.)]
        self.profile_R:list[float] = [(0., 0.), (1., 0.)]
        # self.points:list[tuple[float,float]]  = []
        m = W//4
        h = H//2-100
        dh = 300
        self.points:list[tuple[float,float]]  = [(m, h), (W-m, h), (W-m, h+dh), (m, h+dh)]
        # self.points:list[tuple[float,float]]  = [(m, h), (W//2,h-50), (W-m, h), (W-m, h+100), (W//2,h+100), (m, h+100)]
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
        self.added_left_land = []
        self.added_right_land = []
        #
        self.close_loop = True
        self.modified_point = None
        self.need_refresh = True
        self.width = 16
        self.auto_kerbs = True
        self.params = [] #for load/save
        self.fn = ""
        self.centers:list[V2|None] = [None for _ in self.points]
        self.loop:rad.Loop = rad.Loop(self.points, self.centers)
        self.refs:list[int] = []
        

    def recompute_kerbs(self):
        self.added_turns_L.clear()
        self.added_turns_R.clear()
        self.refresh_if_needed(force=True)
        self.turns_L, self.turns_R = self.detect_kerbs()
        self.turns_L = self.fill_kerbs_holes(self.turns_L)
        self.turns_R = self.fill_kerbs_holes(self.turns_R)


    def save_to_disk(self, fn="") -> None: 
        root = tk.Tk()
        root.withdraw()
        if not fn:
            fn = filedialog.asksaveasfilename(title="Choose a project", filetypes=[("Track files", "*.track2")])
        if fn:
            if not fn.endswith(".track2"):
                fn += ".track2"
            self.fn = fn
            fn = os.path.basename(self.fn)
            print(fn)
            e_filename.set_text(fn, adapt_parent=False)
            attributes = ["points", "added_turns_L", "added_turns_R", "profile_L", "profile_R",
                          "close_loop", "width", "params", "added_left_land", "added_right_land", "centers"]
            self.params = [
                e_avg_land.get_value(),
                e_width_land.get_value(),
                e_cliff.get_value(),
                e_fill_visual.get_value(),
                e_show_land.get_value(),
                e_show_alt.get_value(),
                e_show_kerbs.get_value(),
                e_res.get_value(),
                e_width.get_value(),
                e_auto_kerbs_sensitivity.get_value(),
                e_land_res.get_value(),
                e_autoland_m.get_value(),
                e_autoland_M.get_value(),
                e_kerb_dh.get_value(),
                e_kerb_h0.get_value(),
                e_kerb_w.get_value(),
                e_kerb_h.get_value(),
            ]
            data_to_save = {attr: getattr(self, attr) for attr in attributes}
            with open(fn, 'wb') as f:
                pickle.dump(data_to_save, f)
            


    def load_from_disk(self, fn=None):
        if not fn:
            root = tk.Tk()
            root.withdraw()
            fn = filedialog.askopenfilename(title="Choose a project", filetypes=[("Track files", "*.track2")])
            self.fn = fn
        with open(fn, 'rb') as f:
            data_loaded = pickle.load(f)
        for attr, value in data_loaded.items():
            setattr(self, attr, value)
        try: #new file format
            e_avg_land.set_value(self.params[0])
            e_width_land.set_value(self.params[1])
            e_cliff.set_value(self.params[2])
            e_fill_visual.set_value(self.params[3])
            e_show_land.set_value(self.params[4])
            e_show_alt.set_value(self.params[5])
            e_show_kerbs.set_value(self.params[6])
            e_res.set_value(self.params[7])
            e_width.set_value(self.params[8])
            e_auto_kerbs_sensitivity.set_value(self.params[9])
            e_land_res.set_value(self.params[10])
            e_autoland_m.set_value(self.params[11])
            e_autoland_M.set_value(self.params[12])
            #
            e_kerb_dh.set_value(self.params[13])
            e_kerb_h0.set_value(self.params[14])
            e_kerb_w.set_value(self.params[15])
            e_kerb_h.set_value(self.params[16])
            old_format = False
        except:
            print("Old file format, some parameters may not be loaded")
            old_format = True
        fn = os.path.basename(fn)
        e_filename.set_text(fn+"\n(compatibility mode)"*old_format)
        # self.refresh_if_needed(force=True)
        self.refresh_if_needed()
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
            if len(self.points) >= 3:
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
            prob_next = here.distance_squared_to(next_point) < 1e-6
            prob_prev = here.distance_squared_to(prev_point) < 1e-6
            if prob_next and prob_prev:
                lane.append(here + V2(width, width))
                continue
            if prob_next:
                dir_prev = (here-prev_point).normalize()
                right_p = here + width * dir_prev.rotate(angle)
                lane.append((right_p.x, right_p.y))
                continue
            elif prob_prev:
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

    def write_land_stl(self, name, L:list[tuple], R:list[tuple], left_side):
        # print(len(L), len(R))
        LENGTH = len(L)
        assert len(L) == len(R)
        profile_L = smooth_profile(self.profile_L, LENGTH)
        profile_R = smooth_profile(self.profile_R, LENGTH)
        if left_side:
            profile_L, profile_R = profile_R, profile_L
        factor = e_kerb_w.get_value()
        # print("****", e_kerb_w.get_value(), self.width)
        with open(name+".stl", "w") as f:
            f.write(f"solid {name}\n")
            for i in range(LENGTH):
                # print(profile_L[i][1])
                ip = i%LENGTH
                jp = (i+1)%LENGTH
                q = tri(L[ip], profile_R[ip][1])
                r = tri(L[jp], profile_R[jp][1])
                #
                devers1 = profile_L[ip][1] - profile_R[ip][1]
                devers2 = profile_L[jp][1] - profile_R[jp][1]
                dh1 = factor*devers1
                dh2 = factor*devers2
                s = tri(R[ip], profile_R[ip][1] - dh1)
                t = tri(R[jp], profile_R[jp][1] - dh2)
                if left_side:
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
                else: #flip faces
                    f.write(f"facet normal 0 0 0\nouter loop\n")
                    f.write(f"vertex {q[0]} -{q[1]} {q[2]}\n")
                    f.write(f"vertex {s[0]} -{s[1]} {s[2]}\n")
                    f.write(f"vertex {r[0]} -{r[1]} {r[2]}\n")
                    f.write("endloop\nendfacet\n")
                    f.write(f"facet normal 0 0 0\nouter loop\n")
                    f.write(f"vertex {s[0]} -{s[1]} {s[2]}\n")
                    f.write(f"vertex {t[0]} -{t[1]} {t[2]}\n")
                    f.write(f"vertex {r[0]} -{r[1]} {r[2]}\n")
                    f.write("endloop\nendfacet\n")
            f.write(f"endsolid {name}\n")

    def write_all_stls(self):
        tp.Text("Refresh Land", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        self.refresh_if_needed(force=True, also_force_land=True)
        tp.Text("Writing road", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        self.write_stl("1ROAD1", self.right_lane, self.left_lane)
        smooth_right_land = self.build_lane(self.right_lane, e_kerb_w.get_value()*self.width, 90)
        smooth_left_land = self.build_lane(self.left_lane, e_kerb_w.get_value()*self.width, -90)
        self.write_land_stl("1ROAD_R", self.right_lane, smooth_right_land, left_side=False)
        self.write_land_stl("1ROAD_L", self.left_lane, smooth_left_land, left_side=True)
        # #########################################
        # grid land ############################
        cliff_z = e_cliff.get_value()
        tp.Text("Smoothing profile", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        profile_L = smooth_profile(self.profile_L, len(self.left_lane))
        profile_R = smooth_profile(self.profile_R, len(self.right_lane))
        profile_L = add_devers(profile_L, profile_R, e_kerb_w.get_value()*self.width)
        profile_R = add_devers(profile_R, profile_L, e_kerb_w.get_value()*self.width)
        # Right ############################
        tp.Text("Making GRASS_L_LAND triangles", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        triangles = make_land_triangles(self.left_land, smooth_left_land, profile_L, flip_faces=False, cliffs=cliff_z)
        # triangles = make_land_triangles(self.left_land, self.left_lane, profile_L,flip_faces=False, cliffs=cliff_z)
        tp.Text("Writing GRASS_L_LAND", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        write_triangles(triangles, "1GRASS_L_LAND") #left
        # Right ############################
        tp.Text("Making GRASS_R_LAND triangles", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        triangles = make_land_triangles(self.right_land, smooth_right_land, profile_R, flip_faces=True, cliffs=cliff_z)
        # triangles = make_land_triangles(self.right_land, self.right_lane, profile_R, flip_faces=True, cliffs=cliff_z)
        tp.Text("Writing GRASS_R_LAND", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        write_triangles(triangles, "1GRASS_R_LAND") #right
        ########### 
        tp.Text("Writing Kerbs", 40, (0,0,0)).center_on(screen).draw_and_display_rect((255,)*3)
        self.write_kerbs_stl("1KERB_L1", -90, True)
        self.write_kerbs_stl("1KERB_L2", -90, False)
        self.write_kerbs_stl("1KERB_R1", 90, True)
        self.write_kerbs_stl("1KERB_R2", 90, False)
        #
        save_layout_image()



    def write_kerbs_stl(self, name, angle, parity):
        triangles = []
        base = e_kerb_h0.get_value()
        w = self.width * e_kerb_w.get_value()
        dz = self.width * e_kerb_h.get_value()
        ddz = dz * e_kerb_dh.get_value()
        # ddz = 0
        if "L" in name:
            lane = self.left_lane
            turns = self.fill_kerbs_holes(self.turns_L + self.added_turns_L)
            flip_faces = False
            profile = smooth_profile(self.profile_L, len(self.left_lane))
            profile2 = smooth_profile(self.profile_R, len(self.right_lane))
        else:
            lane = self.right_lane
            turns = self.fill_kerbs_holes(self.turns_R + self.added_turns_R)
            flip_faces = True
            profile = smooth_profile(self.profile_R, len(self.right_lane))
            profile2 = smooth_profile(self.profile_L, len(self.left_lane))
        #################
        number = 0
        for i in turns:
            if number % 2 == parity:
                alt_p = profile[i][1] + base
                p = V2(lane[i])
                j = (i+1)%len(lane)
                alt_q = profile[j][1] + base
                q = V2(lane[j])
                if p.distance_squared_to(q) < 1e-6:
                    continue
                r = p + (q-p).normalize().rotate(angle) * w
                s = q + (q-p).normalize().rotate(angle) * w
                devers_r = (profile2[i][1] - profile[i][1]) * w / self.width
                alt_r = alt_p - devers_r #* dz
                devers_s = (profile2[j][1] - profile[j][1]) * w / self.width
                alt_s = alt_q - devers_s #* dz
                if flip_faces:
                    triangles.append((tri(p,alt_p),tri(s,alt_s+dz),tri(q,alt_q+dz-ddz)))
                    triangles.append((tri(r,alt_r),tri(s,alt_s+dz),tri(p,alt_p)))
                    triangles.append((tri(q,alt_q+dz-ddz),tri(p,alt_p),tri(q,alt_q))) #vertical
                else:
                    triangles.append((tri(p,alt_p),tri(q,alt_q+dz-ddz),tri(s,alt_s+dz)))
                    triangles.append((tri(r,alt_r),tri(p,alt_p),tri(s,alt_s+dz)))
                    triangles.append((tri(q,alt_q+dz-ddz),tri(q,alt_q),tri(p,alt_p))) #vertical
            if i%3 == 0:
                number += 1
        if triangles:
            write_triangles(triangles, name)
        else:
            print("NO KERBS:", name)
            if os.path.exists(name+".stl"):
                os.remove(name+".stl")

    def fill_kerbs_holes(self, turns:list[int]) -> list[int]:
        serie_tolerance = 5 #consecutive holes in the kerbs that can be filled
        serie = 0
        last_i = -2
        turns.sort()
        new_turns = []
        for i in turns:
            if i > last_i:
                new_turns.append(i)
            gap = i - last_i
            if gap == 0:
                serie += 1
            else:
                if gap < serie_tolerance:
                    for j in range(1, gap):
                        new_turns.append((last_i+j)%len(self.left_lane))
                    serie += gap
                else:
                    serie = 0
            last_i = i
        return new_turns
        # nnt = []
        # for loop_index in range(len(new_turns)):
        #     prev = (loop_index-1)%len(new_turns)
        #     turn_here = new_turns[loop_index]
        #     turn_before = new_turns[prev]
        #     if turn_before == turn_here:
        #         assert False
        #     else:
        #         nnt.append(turn_here)
        # return nnt

    def detect_kerbs(self):
        turnsL, turnsR = [], []
        # last_turn = None
        M = e_auto_kerbs_sensitivity.get_value()
        if M == 0:
            return [], []
        else:
            M = 178 + M/100 * 2
        m = 180 - M
        for i in range(1,len(self.left_lane)-1): #skip first and last
            idx_next = (i+1)%len(self.left_lane)
            idx_prev = (i-1)%len(self.left_lane)
            p = V2(self.left_lane[idx_prev])
            h = V2(self.left_lane[i])
            n = V2(self.left_lane[idx_next])
            angle = (h-p).angle_to(n-h)
            # print(i, angle)
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
    
        
    def get_raw_or_spline_point(self, ref_point):
        i, coord = track.closest_point(ref_point, MAGNET1)
        if coord:
            return i, coord, "raw"
        else:
            i, coord = track.closest_point(ref_point, MAGNET2, candidates=self.left_lane)
            if coord:
                return i, coord, "spline"
        return -1, None, None
    
    
    def reset_added_land(self):
        self.added_left_land = []
        self.added_right_land = []
        self.refresh_land()
    
    def refresh_land(self):
        n = e_land_res.get_value()
        self.left_land, self.right_land = spawn_valid_land_points(n, int(n*H/W))
        self.left_land += self.added_left_land
        self.right_land += self.added_right_land

    def refresh_land_light(self):
        f = e_width_land.get_value() / 100 * 10
        # print("light", f, f*self.width)
        self.right_land = self.build_lane(self.right_lane, f*self.width, 90)
        self.left_land = self.build_lane(self.left_lane, f*self.width, -90)
    
track = Track()

def spawn_valid_land_points(res_w, res_h):
    # m = track.width * 3
    m = e_autoland_m.get_value()
    M = e_autoland_M.get_value()
    valid_points_L = []
    valid_points_R = []
    candidates = track.left_lane+track.right_lane
    if not candidates:
        return valid_points_L, valid_points_R
    for x in np.linspace(0, W, res_w):
        for y in np.linspace(0, H, res_h):
            p = V2(x,y)
            coord = min(candidates, key=lambda q: p.distance_squared_to(q))
            d = p.distance_to(coord)
            if m < d < M:
                if coord in track.left_lane:
                    valid_points_L.append(tuple(p))
                    col = (255,0,0)
                elif coord in track.right_lane:
                    valid_points_R.append(tuple(p))
                    col = (0,0,255)
    return valid_points_L, valid_points_R

# def spawn_valid_land_points(res_w, res_h):
#     import random
#     # m = track.width * 3
#     m = e_autoland_m.get_value()
#     M = e_autoland_M.get_value()
#     valid_points_L = []
#     valid_points_R = []
#     candidates = track.left_lane+track.right_lane
#     if not candidates:
#         return valid_points_L, valid_points_R
#     n = res_h * res_w
#     for i in range(n):
#         track_point = V2(random.choice(candidates))
#         angle = random.uniform(0, 360)
#         dist = random.uniform(m, M)
#         p = track_point + V2(dist,0).rotate(angle)
#         coord = min(candidates, key=lambda q: p.distance_squared_to(q))
#         d = p.distance_to(coord)
#         if m < d < M:
#             if coord in track.left_lane:
#                 valid_points_L.append(tuple(p))
#                 col = (255,0,0)
#             elif coord in track.right_lane:
#                 valid_points_R.append(tuple(p))
#                 col = (0,0,255)
#     return valid_points_L, valid_points_R


def change_cat():
    global e_edit, e_help
    cat = e_cat.get_value()
    if not cat:
        cat = "Track"
    e_all.replace_child(e_edit, categories[cat])
    e_edit = categories[cat]
    h = helps_cat.get(cat)
    if h:
        e_all.replace_child(e_help, h)
        e_help = h
        e_help.stick_to(screen, "top", "top")
        e_help.set_invisible(False)
    else:
        e_help.set_invisible(True)
    for e in categories.keys():
        if e != cat:
            categories[e].set_invisible(True, recursive=True)
    e_edit.set_invisible(False, recursive=True)
    e_edit.stick_to(screen, "left", "left", (10,0))
    # categories[cat].set_draggable(True,True,False,False)

def update_all(): #do what you want with the display like in any pygame code you write
    pts, refs = track.loop.to_points(e_res.get_value())
    # pygame.draw.lines(screen, (0,155,0), True, pts, 2)
    track.left_lane = pts
    track.refs = refs
    # for point in pts:
    #     pygame.draw.circle(screen, (0,0,0), point, 2)
    select = e_cat.get_value()
    actual = categories.get(select)
    if actual and not(actual is e_edit):
        change_cat()
    i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1, candidates=track.left_lane)
    prof = track.smoothed_profile_L
    if i >= 0 and i < len(prof):
        e_alt.set_text(f"Alt: {round(prof[i][1])}\nTrack fraction: {round(100*i/len(prof))}%")
        e_alt.set_bottomleft(10, H-e_alt.get_current_height()-10)
    else:
        e_alt.set_bottomright(-10,-10) #quick hide
    #
    track.refresh_if_needed()

def draw_all():
    collided = None
    track_mode = e_cat.get_value() == "Track"
    screen.fill((255,255,255))
    ######
    track.loop = rad.Loop(track.points, track.centers)
    # else:
        # print("turn 0 5", track.loop.turns[5].d, track.loop.turns[5].c)
    # ################# ASPHALT AND LAND ###########################
    fill_color = e_fill_visual.get_value()
    # if e_show_land.get_value():
    if False:
        track.draw_polygon(track.left_land, (0,200,0), 2, fill_color)
        fill_color_right_lane = (0,200,0)
    else:
        fill_color_right_lane = (255,255,255)
    track.draw_polygon(track.left_lane, (200,200,200), 1, fill_color)
    track.draw_polygon(track.right_lane, fill_color_right_lane, 1, fill_color)
    if False:
        track.draw_polygon(track.right_land, (255,255,255), 2, fill_color)
        # for i_p, point in enumerate(track.right_land):
        #     pygame.draw.circle(screen, (0,0,255), point, 2)
    # ################ POINTS ############################
    if track_mode:
        for i_p, point in enumerate(track.points):
            pygame.draw.circle(screen, (0,0,0), point, 6)
    # ###################### PROFILE ############################
    if len(track.smoothed_profile_L) > 4 and (e_show_alt.get_value() or e_cat.get_value() == "Terrain"):
        base_r_h = 20
        n_ticks = 40
        profile = track.smoothed_profile_L
        lane = track.left_lane
        #Draw interpolated bars
        for i in range(0,len(profile), len(profile)//n_ticks):
            alt = profile[i][1]
            rect = pygame.Rect(lane[i], (4,base_r_h+alt))
            pygame.draw.rect(screen, (50,50,50), rect)
        i_prof = 0
        def collide(rect, i_prof):
            if rect.collidepoint(pygame.mouse.get_pos()) and e_cat.get_value() == "Terrain":
                pygame.draw.rect(screen, (255,0,0), rect)
                return i_prof
            return None
        for x,alt in track.profile_L[0:-1]: #last is first so skip it
            idx = int(x * len(profile))
            coord = lane[idx]
            rect = pygame.Rect(coord, (20,base_r_h+alt))
            pygame.draw.rect(screen, (0,200,0), rect)
            pygame.draw.rect(screen, (0,0,0), rect, 1)
            collided = collide(rect, i_prof)
            i_prof += 1
        ##########################################################
        profile = track.smoothed_profile_R
        lane = track.right_lane
        for i in range(0,len(profile), len(profile)//n_ticks):
            alt = profile[i][1]
            rect = pygame.Rect(lane[i], (4,base_r_h+alt))
            pygame.draw.rect(screen, (50,50,50), rect)
        i_prof = 0
        for x,alt in track.profile_R[0:-1]: #last is first so skip it
            idx = int(x * len(profile))
            coord = lane[idx]
            rect = pygame.Rect(coord, (20,base_r_h+alt))
            pygame.draw.rect(screen, (255,255,0), rect)
            pygame.draw.rect(screen, (0,0,0), rect, 1)
            collided_tmp = collide(rect, i_prof)
            if collided_tmp:
                collided = collided_tmp
            i_prof += 1
    ################ TURNS (KERBS) ############################
    if e_show_kerbs.get_value() or e_cat.get_value() == "Kerbs":
        for i in track.turns_L + track.added_turns_L:
            pygame.draw.rect(screen, (255,0,0), pygame.Rect(track.left_lane[i], (6,6)))
        for i in track.turns_R + track.added_turns_R:
            pygame.draw.rect(screen, (0,0,255),  pygame.Rect(track.right_lane[i], (6,6)))
    # #### HANDLE CURSOR ################################################
    i, coord, kind = track.get_raw_or_spline_point(pygame.mouse.get_pos())
    if kind == "raw":
        if track_mode:
            pygame.draw.circle(screen, COLOR_POINT_HOVER, coord, 8)
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
    elif kind == "spline":
        if track_mode: pygame.draw.circle(screen, COLOR_POINT_HOVER, coord, 5)
        pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_CROSSHAIR)
    else:
        pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
    # spawn_valid_land_points(30,20)
    # if e_show_land.get_value():
    # if False:
    #     for i_p, point in enumerate(track.left_land+track.added_left_land):
    #         pygame.draw.circle(screen, (0,0,255), point, 4)
    #     for i_p, point in enumerate(track.right_land+track.added_right_land):
    #         pygame.draw.circle(screen, (255,0,0), point, 4)
    if True:
        for i_p, point in enumerate(track.added_left_land):
            pygame.draw.circle(screen, (0,0,255), point, 4)
        for i_p, point in enumerate(track.added_right_land):
            pygame.draw.circle(screen, (255,0,0), point, 4)
    # e_alt.draw()
    # track.loop.draw(screen)
    if track.left_lane:
        pygame.draw.circle(screen, (255,0,0), track.left_lane[0], 12)
    return collided

def save_layout_image(bck=(0,0,0,0), asphalt=(250,)*3, res=(1920,1080)):
    W, H = res
    pts = track.left_lane + track.right_lane
    #
    mx = min(pts, key=lambda p: p[0])[0]
    Mx = max(pts, key=lambda p: p[0])[0]
    my = min(pts, key=lambda p: p[1])[1]
    My = max(pts, key=lambda p: p[1])[1]
    w = Mx - mx
    h = My - my
    if w > W or h > H:
        factor = W/w if w > h else H/h
        left_lane = [(factor*(x-mx), factor*(y-my)) for x,y in track.left_lane]
        right_lane = [(factor*(x-mx), factor*(y-my)) for x,y in track.right_lane]
    else:
        left_lane = [(x-mx, y-my) for x,y in track.left_lane]
        right_lane = [(x-mx, y-my) for x,y in track.right_lane]
    #
    w = int(w)+1
    h = int(h)+1
    surface = pygame.Surface((w,h), pygame.SRCALPHA)
    surface.fill(bck)
    gfx.filled_polygon(surface, left_lane, asphalt)
    gfx.filled_polygon(surface, right_lane, bck)
    from PIL import Image
    # Convertir la surface Pygame en une image PIL
    raw_str = pygame.image.tostring(surface, 'RGBA')
    image = Image.frombytes('RGBA', (w, h), raw_str)
    # Sauvegarder l'image avec PIL pour préserver la transparence
    image.save("outline.png")

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
        # print(track.profile_L)
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

def shift(pts, dx, dy):
    for i in range(len(pts)):
        pts[i] = (pts[i][0]+dx, pts[i][1]+dy)

# e_res = tp.TogglablesPool("Display", ["Raw points", "Splines 2", "Splines 4", "Splines 16"], 0)
e_res = tp.SliderWithText("Mesh resolution", 0.01, 3, 0.5, 100)
e_res.slider.dragger.at_drag = track.mark_refresh

e_width = tp.SliderWithText("Track width", 1, 64, 16, 100)
e_width.slider.dragger.at_drag = track.mark_refresh

e_avg_land = tp.SliderWithText("Avg Land", 0, 200, 70, 100)
e_avg_land.slider.dragger.at_drag = track.mark_refresh

e_width_land = tp.SliderWithText("Land Width", 0, 100, 30, 100)
e_width_land.slider.dragger.at_drag = track.mark_refresh

e_fill_visual = tp.Labelled("Fill color", tp.Checkbox(True))
e_show_kerbs = tp.Labelled("Kerbs", tp.Checkbox(True))
e_show_alt = tp.Labelled("Altitude", tp.Checkbox(True))
e_show_land = tp.Labelled("Grass", tp.Checkbox(False))

e_savep = tp.Button("Save project as...")
e_savep.at_unclick = track.save_to_disk

e_loadp = tp.Button("Load project")
e_loadp.at_unclick = track.load_from_disk

e_alt = tp.Text("Alt: 0 m\nTrack fraction: 0%")
e_alt.set_bottomleft(50, H-e_alt.get_current_height()-10)

# e_refresh_land = tp.Button("Show terrain mesh points")
# e_refresh_land.at_unclick = track.refresh_land

e_cliff = tp.SliderWithText("Cliff height", -1, 100, 0, 100)
tp.Helper("Elevation difference between the track and the grass. -1 means no difference ('flat' terrain locally).", e_cliff, 30)

e_save = tp.Button("Export STL's")
e_save.at_unclick = track.write_all_stls


e_recompute_kerbs = tp.Button("Remove & recompute kerbs")
e_recompute_kerbs.at_unclick = track.recompute_kerbs

e_auto_kerbs_sensitivity = tp.SliderWithText("Autokerbs\nsensitivity", 0, 100, 50, 100)

e_land_res = tp.SliderWithText("Land resolution", 10, 210, 80, 100)
tp.Helper("Impacts the number of vertices used for the terrain mesh. High values can result is a long processing time.", e_land_res, 30)
e_autoland_m = tp.SliderWithText("Close tolerance", 0., 50., 10, 100)
tp.Helper("How close terrain points can be from the track", e_autoland_m, 30)
e_autoland_M = tp.SliderWithText("Far tolerance", 0., 200., 30., 100)
tp.Helper("How far terrain points can be from the track", e_autoland_M, 30)
e_reset_added_land = tp.Button("Reset user's land points")
e_reset_added_land.at_unclick = track.reset_added_land
tp.Helper("Remove all the points you have added.", e_autoland_M, 30)

e_kerb_w = tp.SliderWithText("Kerb width factor", 0., 1., 0.15, 100)
e_kerb_h0 = tp.SliderWithText("Kerb base height", 0., 0.1, 0., 100, round_decimals=2)
e_kerb_h = tp.SliderWithText("Kerb height factor", 0., 1/50, 1/200, 100, round_decimals=3)
e_kerb_dh = tp.SliderWithText("Kerb dh factor", 0., 2., 0.9, 100)

e_filename = tp.Text("track not saved")

e_box_track = tp.TitleBox("Track",[e_res, e_width])
# e_box_land = tp.TitleBox("Terrain",[e_avg_land, e_width_land, e_cliff, e_land_res,
#                                     e_autoland_m, e_autoland_M, e_reset_added_land, e_refresh_land])
e_box_land = tp.TitleBox("Terrain",[e_cliff, e_land_res, e_autoland_m, e_autoland_M, e_reset_added_land])
# e_box_land.sort_children("grid", ny=1)
e_box_kerb = tp.TitleBox("Kerbs",[e_kerb_w, e_kerb_h0, e_kerb_h, e_kerb_dh, e_auto_kerbs_sensitivity, e_recompute_kerbs])
# e_box_kerb.sort_children("grid", ny=1)
e_box_draw = tp.Group([e_fill_visual, e_show_kerbs, e_show_alt], "h")
# e_box_draw = tp.TitleBox("Display",[e_fill_visual, e_show_kerbs, e_show_alt])
# e_box_draw.sort_children("h")
e_box_draw.set_topright(W-10, 10)
# e_box_disk = tp.Group([e_filename, e_savep, e_loadp, e_save])
e_box_disk = tp.TitleBox("File",[e_filename, e_savep, e_loadp, e_save])

e_help_track = tp.Text("""#col_Right click# on the track to add points. You can drag points to move them.
When hovering a point, press #col_X# to delete it.""".replace("#col_", "#RGB(0,0,255)"))
e_help_track.set_font_rich_text_tag("#")

e_help_terrain = tp.Text("""#col_Left click# to add an elevation constraint at the hovered point.
When hovering an altitude bar, #col_left click# it to edit the constraint.
#col_LCTRL+Left/Right click# to add a left/right land point at the mouse position (useful when terrain mesh is messy).""".replace("#col_", "#RGB(0,0,255)"))
e_help_terrain.set_font_rich_text_tag("#")

e_help_kerbs = tp.Text("""#col_Left/Right click# to add a left/right kerb at the mouse position. Hold #col_<LCTRL># to remove the kerbs""".replace("#col_", "#RGB(0,0,255)"))
e_help_kerbs.set_font_rich_text_tag("#")

e_help_file = tp.Text("""Once the STL files are exported, open them in your favourite 3D editor for texture editing and export as FBX.
In Blender, you may find the script 'blender.py' (provided with this project) very useful for quick updates.""".replace("#col_", "#RGB(0,0,255)"))
e_help_file.set_font_rich_text_tag("#")

e_help = e_help_track

helps_cat = {"Track":e_help_track, "Terrain":e_help_terrain, "Kerbs":e_help_kerbs, "File":e_help_file}







e_cat = tp.TogglablesPool("Edit", ["Track", "Terrain", "Kerbs", "File"], 0)
# e_cat.at_unclick = change_cat
e_cat.set_topleft(10,10)

categories = {
    "File": e_box_disk,
    "Track": e_box_track,
    "Terrain": e_box_land,
    "Kerbs": e_box_kerb,
              }

e_edit = categories["Track"]

e_all = tp.Group([e_cat, e_edit, e_box_draw, e_help], None)
updater = e_all.get_updater()


# track.refresh_if_needed(force=True)
# track.load_from_disk("m93.track")
# track.refresh_if_needed(force=True)

track.smoothed_profile_L = smooth_profile(track.profile_L, len(track.left_lane))
track.smoothed_profile_R = smooth_profile(track.profile_R, len(track.right_lane))

change_cat()

zoom = 1.

def events_terrain(events, mouse_rel, collided):
    cat = e_cat.get_value()
    if not cat == "Terrain":
        return
    ctrl_pressed = pygame.key.get_mods() & pygame.KMOD_CTRL
    for e in events:
        if e.type == pygame.MOUSEBUTTONDOWN:
            if e.button == 3 and ctrl_pressed: #right
                track.added_right_land.append(pygame.mouse.get_pos())
            elif e.button == 1 and ctrl_pressed: #left
                track.added_left_land.append(pygame.mouse.get_pos())
            elif e.button == 1 and collided:
                track.mark_refresh()
                alt_L, alt_R, deletion = choose_alt_i(collided)
                print(alt_L, alt_R, deletion)
                if deletion:
                    print("deleted", collided)
                    return
                else:
                    l, r = track.profile_L[collided], track.profile_R[collided]
                    track.profile_L[collided] = (l[0], alt_L)
                    track.profile_R[collided] = (r[0], alt_R)
            elif e.button == 1:
                track.mark_refresh()
                i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1, candidates=track.left_lane)
                x_elevation = i / len(track.left_lane)
                track.insert_profile_point(x_elevation, track.profile_L)
                track.insert_profile_point(x_elevation, track.profile_R)
                    
                

def events_kerb(events, mouse_rel):
    cat = e_cat.get_value()
    if not cat == "Kerbs":
        return
    for e in events:
        if e.type == pygame.MOUSEMOTION:
            right = pygame.mouse.get_pressed()[2]
            left = pygame.mouse.get_pressed()[0]
            if right: #right
                cand = track.right_lane
                turns = track.added_turns_R
                autoturns = track.turns_R
            elif left: #left
                cand = track.left_lane
                turns = track.added_turns_L
                autoturns = track.turns_L
            if left or right:
                i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1, candidates=cand)
                if i >= 0:
                    if pygame.key.get_mods() & pygame.KMOD_CTRL:
                        if i in turns:
                            turns.remove(i)
                        elif i in autoturns:
                            autoturns.remove(i)
                    elif not(i in turns) and not(i in autoturns):
                        turns.append(i)
                turns = track.fill_kerbs_holes(turns)

def events_track(events, mouse_rel):
    cat = e_cat.get_value()
    if cat and not cat == "Track":
        return
    for e in events:
        if e.type == pygame.MOUSEMOTION:
            if pygame.mouse.get_pressed()[0]: #Drag point
                track.mark_refresh()
                if track.modified_point is None:
                    i, _ = track.closest_point(e.pos, MAGNET1)
                    if i >= 0:
                        track.points[i] = e.pos
                        i_turn = track.loop.i_track_to_loop(i)
                        track.centers[i] = track.loop.turns[i_turn].d
                        track.mark_refresh()
                        track.modified_point = i
                    else:
                        track.modified_point = None
                else:
                    track.points[track.modified_point] = e.pos
                    i_turn = track.loop.i_track_to_loop(track.modified_point)
                    track.centers[track.modified_point] = track.loop.turns[i_turn].d
                    track.mark_refresh()
            else:
                track.modified_point = None
                if pygame.mouse.get_pressed()[1]: #shift track
                    track.mark_refresh()
                    shift(track.points, *mouse_rel)
                    shift(track.added_left_land, *mouse_rel)
                    shift(track.added_right_land, *mouse_rel)
        elif e.type == pygame.MOUSEBUTTONDOWN:
            if e.button == 3: #right click
                track.mark_refresh()
                i, coord = track.closest_point(e.pos, MAGNET1, track.left_lane)
                raw_i = track.refs[i]
                track.points.insert(raw_i, coord)
                track.centers.insert(raw_i, None)
                track.loop = rad.Loop(track.points, track.centers)
            #check wheel mouse
            if e.button == 4 or e.button == 5:
                mp = pygame.mouse.get_pos()
                i, coord = track.closest_point(mp, MAGNET1)
                if i >= 0:
                    assert len(track.points) == len(track.loop.turns)
                    i_turn = track.loop.i_track_to_loop(i)
                    turn = track.loop.turns[i_turn]
                    backup_d = turn.d
                    #if ctrl is pressed:
                    if pygame.key.get_mods() & pygame.KMOD_CTRL:
                        delta_d = 1
                    else:
                        delta_d = 10
                    if e.button == 4:
                        delta_d *= -1
                    turn.d += delta_d
                    if turn.d < 0:
                        turn.d = 0
                    # v = V2(mp) - turn.q()
                    # w = turn.bissec(1000) - turn.q()
                    # if w.length_squared() < 1e-6:
                    #     w = turn.s1.delta.rotate(90)
                    # mouse_proj = v.project(w)
                    # point_on_bissec = turn.q() + mouse_proj
                    # #
                    turn.compute_c(turn.bissec(turn.d))
                    d1 = (turn.s1.ar-turn.s1.br).length()
                    d2 = (turn.s2.ar-turn.s2.br).length()
                    if d1 <= 0 or d2 <= 0:
                        turn.compute_c(turn.bissec(backup_d))
                    track.centers[i] = turn.d
                    track.mark_refresh()
        elif e.type == pygame.KEYDOWN:
            if e.key == pygame.K_x:
                i, _ = track.closest_point(pygame.mouse.get_pos(), MAGNET1)
                if i >= 0:
                    track.points.pop(i)
                track.refresh_if_needed(force=True)

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
        elif e.type == pygame.KEYDOWN:
            if e.key == pygame.K_s:
                if pygame.key.get_mods() & pygame.KMOD_CTRL:
                    track.save_to_disk(track.fn)
                    t = tp.Text("Project saved as "+track.fn, 30)
                    t.center_on(screen)
                    t.draw()
                    pygame.display.flip()
                    tp.pause()
                    # t.launch_alone(func_before=lambda:screen.fill((255,255,255)) and pygame.display.flip())
    a = e_edit.get_rect_with_children().collidepoint(pygame.mouse.get_pos())
    b = e_cat.get_rect_with_children().collidepoint(pygame.mouse.get_pos())
    iprof = draw_all()
    if not(a or b):
        events_track(events, mouse_rel)
        events_terrain(events, mouse_rel, iprof)
        events_kerb(events, mouse_rel)
    #update Thorpy elements and draw them
    updater.update(update_all, events=events,
                   mouse_rel=mouse_rel) #if you dont give a mouse_rel, Thorpy will call pygame mouse_rel() !
    # e_all.update(mouse_rel)
    pygame.display.flip()
pygame.quit()


#TODO: AMELIORER ERGONOMIE RADIUS SET
#TODO: IO

#TODO: material dirty asphalt, water, sand
#TODO: basic meshes placement, as well as start, etc
#TODO: working zoom (pas toucher a track width, vraiment histoire de camera)