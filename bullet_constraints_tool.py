
# This file is part of the Bullet Constraints Tool addon for Blender.

# The Bullet Constraints Tool is free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.

# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this file. If not, see <https://www.gnu.org/licenses/>.

import bpy
import time
import math
from mathutils import Vector
# from bpy.props import *

bl_info = {
    "name": "Bullet Constraints Tool",
    "author": "bashi; port by Timothy Strange",
    "version": (0, 4, 0, 0),
    "blender": (2, 80, 0),
    "location": "Properties",
    "description":
    "Tool to generate constraints.",
    "warning": "Work in progress",
    "wiki_url":
    "https://github.com/timothy-strange/bullet-constraints-tool/wiki",
    "tracker_url":
    "https://github.com/timothy-strange/bullet-constraints-tool/issues",
    "category": "Object"}


class Bullet_Tools(bpy.types.Panel):
    bl_label = "Bullet Constraints Tool"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'physics'

    def draw(self, context):
        layout = self.layout
        scene = context.window_manager.bullet_tool

        # object = context.object
        # if object.type == 'MESH' or 'EMPTY':

        row = layout.row()
        row.operator("bullet.make_constraints", icon="MOD_BUILD")
        row.operator("bullet.x_connect", icon="MOD_SKIN")

        row = layout.row()
        row.prop(scene, "bullet_tool_neighbours")
        row.prop(scene, "bullet_tool_search_radius")

        row = layout.row()
        row.operator("bullet.from_to_constraint", icon="MOD_BUILD")
        row.operator("bullet.update", icon="FILE_REFRESH")

        row = layout.row()
        row.prop(scene, "bullet_tool_gpencil_mode")
        row.prop(scene, "bullet_tool_gpencil_dis")
        row.operator("bullet.gpencil", icon='GREASEPENCIL')

        # Show Object Settings
        row = layout.row()
        row.prop(scene, "bullet_tool_show_obj")
        row.active = False
        if context.window_manager.bullet_tool.bullet_tool_show_obj is True:
            row.active = True
        row.prop(scene, "bullet_tool_friction")
        row.prop(scene, "bullet_tool_use_margin")

        row = layout.row()
        row.active = False
        if context.window_manager.bullet_tool.bullet_tool_show_obj is True:
            row.active = True
        row.prop(scene, "bullet_tool_bounciness")
        row.prop(scene, "bullet_tool_collmargin")

        # Show Constraint Settings
        row = layout.row()
        row.prop(scene, "bullet_tool_show_con")
        row.active = False
        if context.window_manager.bullet_tool.bullet_tool_show_con is True:
            row.active = True
        row.prop(scene, "bullet_tool_Constraint_type")
        # Show Break Options
        row = layout.row()
        row.prop(scene, "bullet_tool_show_break")
        row.active = False
        if context.window_manager.bullet_tool.bullet_tool_show_break is True:
            row.active = True
        row.prop(scene, "bullet_tool_breakable")
        row.prop(scene, "bullet_tool_break_threshold")
        if scene.bullet_tool_multiplier is False:
            row.prop(scene, "bullet_tool_absolute_mass")
        if scene.bullet_tool_absolute_mass is False:
            row.prop(scene, "bullet_tool_multiplier")
        # Show Iterations Options
        row = layout.row()
        row.prop(scene, "bullet_tool_show_it")
        row.active = False
        if context.window_manager.bullet_tool.bullet_tool_show_it is True:
            row.active = True
        row.prop(scene, "bullet_tool_over_iteration")
        row.prop(scene, "bullet_tool_iteration")
        # For Constraint Types

        # Show Iterations Options
        row = layout.row()
        row.prop(scene, "bullet_tool_show_lim")
        ac = False
        row.active = False
        if context.window_manager.bullet_tool.bullet_tool_show_lim is True:
            ac = True
            row.active = True

        if (context.window_manager.bullet_tool.bullet_tool_Constraint_type
                == 'HINGE'):
            col = layout.column(align=True)
            col.active = False
            if ac is True:
                col.active = True
            col.label("Limits:")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_ang_z", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_ang_z)
            sub.prop(scene, "bullet_tool_limit_ang_z_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_ang_z_upper", text="Upper")

        elif (context.window_manager.bullet_tool.bullet_tool_Constraint_type
                == 'SLIDER'):
            col = layout.column(align=True)
            col.active = False
            if ac is True:
                col.active = True
            col.label("Limits:")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_lin_x", text='X Axis',
                     toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_ang_z)
            sub.prop(scene, "bullet_tool_limit_lin_x_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_lin_x_upper", text="Upper")

        elif (context.window_manager.bullet_tool.bullet_tool_Constraint_type
              == 'PISTON'):
            col = layout.column(align=True)
            col.active = False
            if ac is True:
                col.active = True
            col.label("Limits:")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_lin_x", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_lin_x)
            sub.prop(scene, "bullet_tool_limit_lin_x_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_lin_x_upper", text="Upper")

            col = layout.column(align=True)
            col.active = False
            if ac is True:
                col.active = True

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_ang_x", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_ang_x)
            sub.prop(scene, "bullet_tool_limit_ang_x_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_ang_x_upper", text="Upper")

        elif (context.window_manager.bullet_tool.bullet_tool_Constraint_type
                in {'GENERIC', 'GENERIC_SPRING'}):
            col = layout.column(align=True)
            col.active = False
            if ac is True:
                col.active = True
            col.label("Limits:")

            row = col.row()

            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_lin_x", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_lin_x)
            sub.prop(scene, "bullet_tool_limit_lin_x_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_lin_x_upper", text="Upper")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_lin_y", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_lin_y)
            sub.prop(scene, "bullet_tool_limit_lin_y_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_lin_y_upper", text="Upper")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_lin_z", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_lin_z)
            sub.prop(scene, "bullet_tool_limit_lin_z_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_lin_z_upper", text="Upper")

            col = layout.column(align=True)
            col.active = False
            if ac is True:
                col.active = True

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_ang_x", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_ang_x)
            sub.prop(scene, "bullet_tool_limit_ang_x_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_ang_x_upper", text="Upper")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_ang_y", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_ang_y)
            sub.prop(scene, "bullet_tool_limit_ang_y_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_ang_y_upper", text="Upper")

            row = col.row()
            sub = row.row()
            sub.scale_x = 0.5
            sub.prop(scene, "bullet_tool_use_limit_ang_z", toggle=True)
            sub = row.row()
            sub.active = (context.window_manager.bullet_tool
                          .bullet_tool_use_limit_ang_z)
            sub.prop(scene, "bullet_tool_limit_ang_z_lower", text="Lower")
            sub.prop(scene, "bullet_tool_limit_ang_z_upper", text="Upper")

            if (context.window_manager.bullet_tool.bullet_tool_Constraint_type
                    == 'GENERIC_SPRING'):
                col = layout.column(align=True)
                col.active = False
                if ac is True:
                    col.active = True
                col.label("Springs:")

                row = col.row()
                sub = row.row()
                sub.scale_x = 0.1
                sub.prop(scene, "bullet_tool_use_spring_x", toggle=True,
                         text="X")
                sub = row.row()
                sub.active = (context.window_manager.bullet_tool
                              .bullet_tool_use_spring_x)
                sub.prop(scene, "bullet_tool_spring_stiffness_x")
                sub.prop(scene, "bullet_tool_spring_damping_x")

                row = col.row()
                sub = row.row()
                sub.scale_x = 0.1
                sub.prop(scene, "bullet_tool_use_spring_y", toggle=True,
                         text="Y")
                sub = row.row()
                sub.active = (context.window_manager.bullet_tool
                              .bullet_tool_use_spring_y)
                sub.prop(scene, "bullet_tool_spring_stiffness_y")
                sub.prop(scene, "bullet_tool_spring_damping_y")

                row = col.row()
                sub = row.row()
                sub.scale_x = 0.1
                sub.prop(scene, "bullet_tool_use_spring_z", toggle=True,
                         text="Z")
                sub = row.row()
                sub.active = (context.window_manager.bullet_tool
                              .bullet_tool_use_spring_z)
                sub.prop(scene, "bullet_tool_spring_stiffness_z")
                sub.prop(scene, "bullet_tool_spring_damping_z")

        layout.operator("bullet.ground_connect", icon='UV_VERTEXSEL')

        row = layout.row()

        row.operator("bullet.remove_constraints", icon="X")


# Octree START
# KDTree implementation.
# Features:
# ~ nearest neighbours search
# Matej Drame [matej.drame@gmail.com]

__version__ = "1r11.1.2010"
__all__ = ["KDTree"]


def square_distance(pointA, pointB):
    # squared euclidean distance
    distance = 0
    dimensions = len(pointA)  # assumes both points have the same dimensions
    for dimension in range(dimensions):
        distance += (pointA[dimension] - pointB[dimension]) ** 2
    return distance


class KDTreeNode():
    def __init__(self, point, left, right):
        self.point = point
        self.left = left
        self.right = right

    def is_leaf(self):
        return (self.left is None and self.right is None)


class KDTreeNeighbours():
    # Internal structure used in nearest-neighbours search.

    def __init__(self, query_point, t):
        self.query_point = query_point
        self.t = t  # neighbours wanted
        self.largest_distance = 0  # squared
        self.current_best = []

    def calculate_largest(self):
        if self.t >= len(self.current_best):
            self.largest_distance = self.current_best[-1][1]
        else:
            self.largest_distance = self.current_best[self.t - 1][1]

    def add(self, point):
        sd = square_distance(point, self.query_point)
        # run through current_best, try to find appropriate place
        for i, e in enumerate(self.current_best):
            if i == self.t:
                return  # enough neighbours, this one is farther, forget it
            if e[1] > sd:
                self.current_best.insert(i, [point, sd])
                self.calculate_largest()
                return
        # append it to the end otherwise
        self.current_best.append([point, sd])
        self.calculate_largest()

    def get_best(self):
        return [element[0] for element in self.current_best[:self.t]]


class KDTree():
    #  KDTree implementation.
    #    Example usage:
    #        from kdtree import KDTree
    #        data = <load data> # iterable of points (which are also iterable,
    #                           # same length)
    #        point = <the point of which neighbours we're looking for>
    #        tree = KDTree.construct_from_data(data)
    #        nearest = tree.query(point, t=4) # find nearest 4 points

    def __init__(self, data):
        def build_kdtree(point_list, depth):
            # code based on wikipedia article:
            # http://en.wikipedia.org/wiki/Kd-tree
            if not point_list:
                return None

            # select axis based on depth so that axis cycles through all valid
            # values. assumes all points have the same dimension
            axis = depth % len(point_list[0])
            # print(axis)
            # sort point list and choose median as pivot point,
            # TODO: better selection method, linear-time selection,
            # distribution
            point_list.sort(key=lambda point: point[axis])
            median = int(len(point_list) / 2)  # choose median

            # create node and recursively construct subtrees
            node = KDTreeNode(point=point_list[median],
                              left=build_kdtree(
                                  point_list[0:median], depth + 1),
                              right=build_kdtree(point_list[median + 1:],
                                                 depth + 1))
            return node

        self.root_node = build_kdtree(data, depth=0)

    @staticmethod
    def construct_from_data(data):
        tree = KDTree(data)
        return tree

    def query(self, query_point, t=1):
        # statistics = {'nodes_visited': 0, 'far_search': 0,
        #              'leafs_reached': 0}

        def nn_search(node, query_point, t, depth, best_neighbours):
            if node is None:
                return

            # statistics['nodes_visited'] += 1

            # if we have reached a leaf, let's add to current best neighbours,
            # (if it's better than the worst one or if there is not enough
            # neighbours)
            if node.is_leaf():
                # statistics['leafs_reached'] += 1
                best_neighbours.add(node.point)
                return

            # this node is no leaf

            # select dimension for comparison (based on current depth)
            axis = depth % len(query_point)

            # figure out which subtree to search
            near_subtree = None  # near subtree
            # far subtree (perhaps we'll have to traverse it as well)
            far_subtree = None

            # compare query_point and point of current node in selected
            # dimension and figure out which subtree is farther than the other
            if query_point[axis] < node.point[axis]:
                near_subtree = node.left
                far_subtree = node.right
            else:
                near_subtree = node.right
                far_subtree = node.left

            # recursively search through the tree until a leaf is found
            nn_search(near_subtree, query_point, t, depth + 1, best_neighbours)

            # while unwinding the recursion, check if the current node
            # is closer to query point than the current best,
            # also, until t points have been found, search radius is infinity
            best_neighbours.add(node.point)

            # check whether there could be any points on the other side of the
            # splitting plane that are closer to the query point than the
            # current best
            if ((node.point[axis] - query_point[axis]) ** 2
                    < best_neighbours.largest_distance):
                # statistics['far_search'] += 1
                nn_search(far_subtree, query_point, t,
                          depth + 1, best_neighbours)

            return

        # if there's no tree, there's no neighbors
        if self.root_node is not None:
            neighbours = KDTreeNeighbours(query_point, t)
            nn_search(self.root_node, query_point, t,
                      depth=0, best_neighbours=neighbours)
            result = neighbours.get_best()
        else:
            result = []

        # print statistics
        # print(result)
        return result


# Octree END


def KDTree_make(objs):
    # objs = bpy.context.selected_objects

    # Make dict with Loc as name
    objects = {}
    # Make location List for KDTree
    loc_list = []

    for obj in objs:
        int_loc = (int(obj.location[0] * 1000000),
                   int(obj.location[1] * 1000000),
                   int(obj.location[2] * 1000000))
        objects[int_loc] = obj
        loc_list.append(int_loc)

    # Make KDTree
    tree = KDTree.construct_from_data(loc_list)

    return tree, objects


def KDTree_near(location, objects, tree, neighbours, radius):
    loc = Vector((int(location[0] * 1000000),
                  int(location[1] * 1000000),
                  int(location[2] * 1000000)))

    # t = bpy.context.scene.get('bullet_tool_neighbours', "3.0")
    nearest = tree.query(query_point=loc, t=neighbours)  # Set Nearest Amount

    # Convert Int back to Obj Name + Filter < Distance
    nearestObjects = []
    dist_list = []
    for n in nearest:
        p1 = location
        p2 = objects[n].location
        v = p1 - p2
        dist = v.length
        if dist <= float(radius):
            nearestObjects.append(objects[n])
            dist_list.append(dist)
            # print(dist)

    # print(nearestObjects)
    return nearestObjects, dist_list


def nearestFunction(point, objs):
    nearestObj = None
    dist = 0.0
    old_dist = -1.0
    for obj in objs:
        if obj.type == 'MESH':
            mesh = bpy.data.meshes[obj.data.name]
            for verts in mesh.vertices:
                # get the distance of all global vertices
                dist = (point - (obj.matrix_world @ verts.co)).length
                # for first object
                if old_dist < 0:
                    old_dist = dist
                    nearestObj = obj
                    break
                    # for all other objects
                if dist < old_dist:
                    old_dist = dist
                    nearestObj = obj
                    break
    return nearestObj, dist


# Add Constraints for Bullet Viewport Branch
def constraint_empty(loc, ob1, ob2):
    if ob1.type and ob2.type == 'MESH':
        bpy.context.view_layer.objects.active = ob1
        if len(ob1.data.polygons) is not 0:
            bpy.ops.rigidbody.object_add(type='ACTIVE')

        bpy.context.view_layer.objects.active = ob2
        if len(ob2.data.polygons) is not 0:
            bpy.ops.rigidbody.object_add(type='ACTIVE')

    empty = bpy.data.objects.new("constraint_" + ob1.name + ob2.name, None)

    bpy.context.collection.objects.link(empty)
    empty.location = loc
    bpy.context.view_layer.objects.active = empty
    # bpy.ops.object.empty_add(location=loc)
    # empty = bpy.context.object
    # empty.name = "constraint_"+ob1.name +ob2.name
    empty.empty_display_size = 0.2
    bpy.ops.rigidbody.constraint_add(
        type=str(bpy.context.window_manager.bullet_tool
                 .bullet_tool_Constraint_type))
    empty.rigid_body_constraint.object1 = ob1
    empty.rigid_body_constraint.object2 = ob2
    empty.rigid_body_constraint.use_breaking = (bpy.context.window_manager
                                                .bullet_tool
                                                .bullet_tool_breakable)

    update(bpy.context.selected_objects)  # Check This


def constraint_rigid_viewport(obj, ob1, ob2):

    bpy.context.view_layer.objects.active = obj

    # bpy.ops.object.constraint_add(type='RIGID_BODY_JOINT')
    # bpy.ops.rigidbody.connect(con_type='FIXED', pivot_type='CENTER')
    if obj.type == 'MESH' or 'Empty':
        bpy.ops.rigidbody.constraint_add(
            type=str(bpy.context.window_manager.bullet_tool
                     .bullet_tool_Constraint_type))
        obj.rigid_body_constraint.object1 = ob1
        obj.rigid_body_constraint.object2 = ob2
        obj.rigid_body_constraint.use_breaking = (bpy.context.window_manager
                                                  .bullet_tool
                                                  .bullet_tool_breakable)

        # obj.rigid_body_constraint.override_solver_iterations = True
        # obj.rigid_body_constraint.num_solver_iterations = (bpy.context.scene
        #                                                   .bullet_tool_iteration)

        # Hack
        # obj.rigid_body.collision_shape='MESH'
        # obj.rigid_body.collision_shape='CONVEX_HULL'

    bpy.context.view_layer.objects.active = ob2


def add_constraints(list):
    # bpy.context.scene.rigidbody_world.steps_per_second = 60
    # bpy.context.scene.rigidbody_world.num_solver_iterations =20
    for obj in list:
        # print(list)
        list2 = list
        list2.remove(obj)
        # print(obj)
        nearestObj, dist = nearestFunction(obj.location, list2)
        if nearestObj is not 0:

            # constraint_rigid_viewport(obj, obj, nearestObj)
            # print(len(list), x)
            # x+=1

            # nearestObj, dist = nearestFunction(ground.matrix_world*vertex.co,
            #                                    objs)
            # Limit Distance
            if dist < (bpy.context.window_manager.bullet_tool
                       .bullet_tool_search_radius):
                constraint_rigid_viewport(obj, obj, nearestObj)

        list2 = []


def make_constraints():
    list = bpy.context.selected_objects
    for obj in bpy.context.selected_objects:
        if obj.type == 'MESH':
            bpy.context.view_layer.objects.active = obj
            if len(obj.data.polygons) is not 0:
                if obj.rigid_body:
                    add_constraints(list)
                else:
                    bpy.ops.rigidbody.object_add(type='ACTIVE')
                    add_constraints(list)
                    # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP',
                    #                         iterations=1)


# Grease Pencil function

def GP_tree(ob, objs, tree):
    order = {}
    attributes = []
    objects = []

    neighbours = int(bpy.context.window_manager.bullet_tool.get(
        "bullet_tool_neighbours", "3"))
    neighbours *= 10

    radius = bpy.context.window_manager.bullet_tool.get(
        "bullet_tool_gpencil_dis", "1.0")

    for layer in ob.grease_pencil.layers:
        color = layer.color
        for stroke in layer.active_frame.strokes:
            for point in stroke.points:
                nearestObjects, dist_list = KDTree_near(
                    point.co, objs, tree, neighbours, radius)
                # objs.remove(nearestObj)
                x = 0
                for nearestObj in nearestObjects:
                    dist = dist_list[x]
                    if nearestObj in order:
                        print('Skip! Already in List')
                    else:
                        attributes.append(dist)
                        attributes.append(color)
                        order[nearestObj] = attributes
                        attributes = []
                        objects.append(nearestObj)
                        # order.append(nearestObj)
                    x += 1

    # print(order)
    return objects, order

    # return objs


# Restore Selection
def restore(ob, objs):
    bpy.ops.object.select_all(action='DESELECT')

    for obj in objs:
        obj.select_set(state=True)
    bpy.context.view_layer.objects.active = ob
    # ob.select_set(state=True)


class OBJECT_OT_MakeConstraints(bpy.types.Operator):
    bl_idname = "bullet.make_constraints"
    bl_label = "Single Constraints"

    bl_description = "Add Constraints to Selected. Connect Nearest together. \
1 Constraint per Object"

    def execute(self, context):
        ob = context.object
        objs = context.selected_objects

        make_constraints()
        update(bpy.context.selected_objects)

        restore(ob, objs)

        return {'FINISHED'}


class OBJECT_OT_Bullet_X_Connect(bpy.types.Operator):
    bl_idname = "bullet.x_connect"
    bl_label = "X Constraints"

    bl_description = "KDTree. Constraints between Objects. Uses Neighbour \
                      Limit, Search Radius."

    def execute(self, context):

        time_start = time.time()

        objs = context.selected_objects
        obj = context.object

        obs = []
        for ob in objs:
            if ob.type == 'MESH':
                obs.append(ob)
        tree, objects = KDTree_make(obs)

        avoid_double = True  # <--

        neighbours = context.window_manager.bullet_tool.get(
            "bullet_tool_neighbours", int(3))
        dist = context.window_manager.bullet_tool.get(
            "bullet_tool_search_radius", "0.5")

        for ob in obs:  # Try Unify Code
            if ob.type == 'MESH':
                nearestObjects, dist_list = KDTree_near(
                    ob.location, objects, tree, neighbours, dist)
                print(nearestObjects)
                for obT in nearestObjects:
                    if obT == ob:
                        print('same')
                    else:
                        print(obT)
                        loc = 1 / 2 * (ob.location + obT.location)
                        # Name check if Constraint already exists:
                        if avoid_double is True:
                            if ("constraint_" + ob.name + obT.name
                                    in context.scene.objects):
                                print("Constraint already exists")
                            elif ("constraint_" + obT.name + ob.name
                                  in context.scene.objects):
                                print("Constraint already exists")
                            else:
                                constraint_empty(loc, ob, obT)
                        else:
                            constraint_empty(loc, ob, obT)
                            # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP',
                            #                         iterations=1)
        restore(obj, obs)
        update(objs)

        print(time.time() - time_start)

        return {'FINISHED'}


class OBJECT_OT_FromToConstraint(bpy.types.Operator):
    bl_idname = "bullet.from_to_constraint"
    bl_label = "Constraint Selection2Active"

    bl_description = "Constraint Selected to Active"

    def execute(self, context):
        # Get SelectionOrder
        objs = bpy.context.selected_objects
        ob1 = bpy.context.active_object
        objs.remove(ob1)
        # ob2=objs[0]

        for ob in objs:
            loc = 1 / 2 * (ob.location + ob1.location)
            constraint_empty(loc, ob, ob1)

        update(objs)

        return {'FINISHED'}


def update(objs):
    wm = bpy.context.window_manager

    def up_rigid_body():
        obj.rigid_body.use_deactivation = False
        orb = obj.rigid_body

        if wm.bullet_tool.bullet_tool_show_obj is True:
            orb.use_margin = wm.bullet_tool.bullet_tool_use_margin
            orb.collision_margin = wm.bullet_tool.bullet_tool_collmargin
            orb.restitution = wm.bullet_tool.bullet_tool_bounciness
            orb.friction = wm.bullet_tool.bullet_tool_friction

    def up_rigid_constraint():
        orbc = obj.rigid_body_constraint
        if wm.bullet_tool.bullet_tool_show_break is True:
            orbc.use_breaking = wm.bullet_tool.bullet_tool_breakable

            # Mass
            if wm.bullet_tool.bullet_tool_absolute_mass is True:
                orbc.breaking_threshold = (wm.bullet_tool
                                           .bullet_tool_break_threshold)
            elif wm.bullet_tool.bullet_tool_multiplier is True:
                print('Multiply')
                orbc.breaking_threshold *= (wm.bullet_tool
                                            .bullet_tool_break_threshold)
            elif orbc.object1 and orbc.object2:
                if orbc.object1.type == 'MESH':
                    if orbc.object2.type == 'MESH':
                        if orbc.object1.rigid_body:
                            if orbc.object2.rigid_body:

                                orbc.breaking_threshold = (
                                    ((orbc.object1.rigid_body.mass
                                      + orbc.object2.rigid_body.mass) / 2)
                                    * (wm.bullet_tool
                                       .bullet_tool_break_threshold))
                            else:
                                orbc.breaking_threshold = \
                                    wm.bullet_tool.bullet_tool_break_threshold

        if wm.bullet_tool.bullet_tool_show_it is True:
            orbc.override_solver_iterations = \
                wm.bullet_tool.bullet_tool_over_iteration
            orbc.num_solver_iterations = wm.bullet_tool.bullet_tool_iteration

        if wm.bullet_tool.bullet_tool_show_con is True:
            orbc.type = wm.bullet_tool.bullet_tool_Constraint_type

        if wm.bullet_tool.bullet_tool_show_lim is True:
            orbc.use_limit_ang_x = wm.bullet_tool.bullet_tool_use_limit_ang_x
            orbc.limit_ang_x_lower = math.radians(
                wm.bullet_tool.bullet_tool_limit_ang_x_lower)
            orbc.limit_ang_x_upper = math.radians(
                wm.bullet_tool.bullet_tool_limit_ang_x_upper)

            orbc.use_limit_ang_y = wm.bullet_tool.bullet_tool_use_limit_ang_y
            orbc.limit_ang_y_lower = math.radians(
                wm.bullet_tool.bullet_tool_limit_ang_y_lower)
            orbc.limit_ang_y_upper = math.radians(
                wm.bullet_tool.bullet_tool_limit_ang_y_upper)

            orbc.use_limit_ang_z = wm.bullet_tool.bullet_tool_use_limit_ang_z
            orbc.limit_ang_z_lower = math.radians(
                wm.bullet_tool.bullet_tool_limit_ang_z_lower)
            orbc.limit_ang_z_upper = math.radians(
                wm.bullet_tool.bullet_tool_limit_ang_z_upper)

            orbc.use_limit_lin_x = wm.bullet_tool.bullet_tool_use_limit_lin_x
            orbc.limit_lin_x_lower = \
                wm.bullet_tool.bullet_tool_limit_lin_x_lower
            orbc.limit_lin_x_upper = \
                wm.bullet_tool.bullet_tool_limit_lin_x_upper

            orbc.use_limit_lin_y = \
                wm.bullet_tool.bullet_tool_use_limit_lin_y
            orbc.limit_lin_y_lower = \
                wm.bullet_tool.bullet_tool_limit_lin_y_lower
            orbc.limit_lin_y_upper = \
                wm.bullet_tool.bullet_tool_limit_lin_y_upper

            orbc.use_limit_lin_z = \
                wm.bullet_tool.bullet_tool_use_limit_lin_z
            orbc.limit_lin_z_lower = \
                wm.bullet_tool.bullet_tool_limit_lin_z_lower
            orbc.limit_lin_z_upper = \
                wm.bullet_tool.bullet_tool_limit_lin_z_upper

            orbc.use_spring_x = \
                wm.bullet_tool.bullet_tool_use_spring_x
            orbc.spring_stiffness_x = \
                wm.bullet_tool.bullet_tool_spring_stiffness_x
            orbc.spring_damping_x = wm.bullet_tool.bullet_tool_spring_damping_x

            orbc.use_spring_y = wm.bullet_tool.bullet_tool_use_spring_y
            orbc.spring_stiffness_y = \
                wm.bullet_tool.bullet_tool_spring_stiffness_y
            orbc.spring_damping_y = wm.bullet_tool.bullet_tool_spring_damping_y

            orbc.use_spring_z = wm.bullet_tool.bullet_tool_use_spring_z
            orbc.spring_stiffness_z = \
                wm.bullet_tool.bullet_tool_spring_stiffness_z
            orbc.spring_damping_z = wm.bullet_tool.bullet_tool_spring_damping_z

    def empty_size(empty):
        size = 0.5

        if empty.rigid_body_constraint.type == 'FIXED':
            empty.scale = (size, size, size)
            empty.empty_display_size = size
            empty.empty_display_type = 'PLAIN_AXES'
        if empty.rigid_body_constraint.type == 'POINT':
            empty.scale = (size, size, size)
            empty.empty_display_size = size
            empty.empty_display_type = 'SPHERE'
        if empty.rigid_body_constraint.type == 'HINGE':
            empty.scale = (0.0, 0.0, size * 3)
            empty.empty_display_size = size
            empty.empty_display_type = 'PLAIN_AXES'
        if empty.rigid_body_constraint.type == 'SLIDER':
            empty.scale = (size * 3, 0.0, 0.0)
            empty.empty_display_size = size
            empty.empty_display_type = 'PLAIN_AXES'
        if empty.rigid_body_constraint.type == 'PISTON':
            empty.scale = (size * 3, 0.0, 0.0)
            empty.empty_display_size = size
            empty.empty_display_type = 'PLAIN_AXES'
        if empty.rigid_body_constraint.type == 'GENERIC':
            empty.scale = (size, size, size)
            empty.empty_display_size = size
            empty.empty_display_type = 'PLAIN_AXES'
        if empty.rigid_body_constraint.type == 'GENERIC_SPRING':
            empty.scale = (size, size, size)
            empty.empty_display_size = size
            empty.empty_display_type = 'PLAIN_AXES'

    for obj in objs:
        # Only Allow Mesh or Empty
        if obj.type == 'MESH':
            # Check for Rigibody
            if obj.rigid_body:
                up_rigid_body()

            # Check for Constraints
            if obj.rigid_body_constraint:
                up_rigid_constraint()

                # obj.rigid_body_constraint.disable_collisions=0
        elif obj.type == 'EMPTY':
            if obj.rigid_body_constraint:
                up_rigid_constraint()
                empty_size(obj)


class OBJECT_OT_Bullet_Update(bpy.types.Operator):
    bl_idname = "bullet.update"
    bl_label = "Update Selected"

    bl_description = "Update Settings to Selected"

    def execute(self, context):
        update(bpy.context.selected_objects)

        return {'FINISHED'}


class OBJECT_OT_Bullet_GPencil(bpy.types.Operator):
    bl_idname = "bullet.gpencil"
    bl_label = "GPencil"

    bl_description = "Update/Activate by Grease Pencil Strokes(active Object) \
        within Search Radius"

    def execute(self, context):

        ob1 = bpy.context.object
        objs = context.selected_objects

        tree, objects = KDTree_make(objs)

        gp_objs, order = GP_tree(ob1, objects, tree)

        # Get SelectionOrder
        # objs = bpy.context.selected_objects
        # ob1=bpy.context.active_object
        # if ob1 in objs:
        #    objs.remove(ob1)
        # ob2=objs[0]

        # obs = list(gp_objs.keys())

        # for i in range(0, len(gp_objs)):

        #     if i+2 > len(gp_objs):
        #         break
        #     constraint_rigid_viewport(gp_objs[i], gp_objs[i], gp_objs[i+1])

        # Enable Physics for Objects
        for ob in gp_objs:
            bpy.context.view_layer.objects.active = ob
            if ob.type == 'MESH':
                if ob.rigid_body:
                    ob.rigid_body.type = 'ACTIVE'
                else:
                    if len(ob.data.polygons) is not 0:
                        bpy.ops.rigidbody.object_add(type='ACTIVE')

        neighbours = int(context.window_manager.bullet_tool.get(
            "bullet_tool_neighbours", "3"))
        neighbours += 1
        # Neighbours *= 10
        dist = context.window_manager.bullet_tool.get(
            "bullet_tool_search_radius", "0.5")
        avoid_double = True

        if context.window_manager.bullet_tool.bullet_tool_gpencil_mode is True:

            tree, objects = KDTree_make(gp_objs)

            for ob in gp_objs:  # Try Unify Code
                if ob.type == 'MESH':
                    nearestObjects, dist_list = KDTree_near(
                        ob.location, objects, tree, neighbours, dist)
                    print(nearestObjects)
                    for obT in nearestObjects:
                        if obT == ob:
                            print('same')
                        else:
                            print(obT)
                            loc = 1 / 2 * (ob.location + obT.location)
                            # Name check if Constraint already exists:
                            if avoid_double is True:
                                if ("constraint_" + ob.name + obT.name in
                                        context.scene.objects):
                                    print("Constraint already exists")
                                elif ("constraint_" + obT.name + ob.name in
                                        context.scene.objects):
                                    print("Constraint already exists")
                                else:
                                    constraint_empty(loc, ob, obT)
                            else:
                                constraint_empty(loc, ob, obT)
                                # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP',
                                #                         iterations=1)

        print(gp_objs)
        update(gp_objs)

        restore(ob1, objs)

        return {'FINISHED'}


# def set_ground(ob):

class OBJECT_OT_Bullet_Ground_connect(bpy.types.Operator):
    bl_idname = "bullet.ground_connect"
    bl_label = "(WIP) Ground Connect"

    bl_description = "Constraint Ground (Vertices) to Selected"

    def execute(self, context):

        objs = context.selected_objects

        # Setup Ground
        ground = context.object  # del bpy

        context.view_layer.objects.active = ground  # del bpy
        objs.remove(ground)

        tree, objects = KDTree_make(objs)
        # print(tree)

        if ground.rigid_body:
            ground.rigid_body.mass = 1000
        else:

            bpy.ops.rigidbody.object_add(type='PASSIVE')
            ground.rigid_body.mass = 1000

        print('Ground Connect Start')

        n = context.window_manager.bullet_tool.bullet_tool_neighbours
        d = context.window_manager.bullet_tool.bullet_tool_search_radius

        avoid_double = 1

        for v in ground.data.vertices:

            if len(objs) > 0:

                nearestObj, dist = KDTree_near(
                    ground.matrix_world * v.co, objects, tree, n, d)
                # print(nearestObj)
                nearestObj, dist = nearestFunction(ground.matrix_world
                                                   * v.co, objs)

                for ob in nearestObj:
                    loc = 1 / 2 * ((ground.matrix_world * v.co) + ob.location)
                    # Name check if Constraint already exists:
                    if avoid_double is True:
                        if ("constraint_" + ob.name + ground.name in
                                context.scene.objects):
                            print("Constraint already exists")
                        elif ("constraint_" + ground.name + ob.name in
                                context.scene.objects):
                            print("Constraint already exists")
                        else:
                            constraint_empty(
                                ground.matrix_world * v.co, ob, ground)
                    else:
                        constraint_empty(loc, ob, ground)

        update(objs)
        restore(ground, objs)

        return {'FINISHED'}


class OBJECT_OT_Bullet_remove_constraints(bpy.types.Operator):
    bl_idname = "bullet.remove_constraints"
    bl_label = "Remove Constraints"
    bl_description = "Remove Constraints on Selected"

    def execute(self, context):

        objs = context.selected_objects
        # bpy.ops.object.select_all(action='DESELECT')
        for obj in objs:
            if obj.type == 'MESH':
                if obj.rigid_body_constraint:
                    context.view_layer.objects.active = obj
                    if bpy.ops.rigidbody.constraint_remove.poll():
                        bpy.ops.rigidbody.constraint_remove()
                    else:
                        # Sometimes the constraint_remove poll() fails
                        # with a complaint that the context is wrong.
                        # This may be because the button to run this
                        # function is in the properties window so
                        # the context will be PROPERTIES instead of
                        # VIEW_3D. Nevertheless the problem seems to
                        # only arise when objects without constraints
                        # are in the selection group when this function
                        # is called. We can get around the problem by
                        # setting context to VIEW_3D then setting it
                        # back to PROPERTIES immediately.
                        bpy.context.area.type = 'VIEW_3D'
                        if bpy.ops.rigidbody.constraint_remove.poll():
                            bpy.ops.rigidbody.constraint_remove()
                        bpy.context.area.type = 'PROPERTIES'

            if obj.type == 'EMPTY':
                print('is Empty')
                if obj.rigid_body_constraint:
                    context.view_layer.objects.active = obj
                    bpy.ops.object.delete(use_global=False)

        return {'FINISHED'}


# Properties Test
class BulletToolProps(bpy.types.PropertyGroup):

    bool = bpy.props.BoolProperty
    float = bpy.props.FloatProperty
    int = bpy.props.IntProperty

    bullet_tool_show_obj: bool(
        name="", default=False, description='Enable Object Settings Update')
    bullet_tool_show_con: bool(
        name="", default=False, description='Enable Type Settings Update')
    bullet_tool_show_break: bool(
        name="", default=False, description='Enable Break Threshold Update')
    bullet_tool_show_it: bool(name="", default=False,
                              description='Enable Override Iterations Update')
    bullet_tool_show_lim: bool(
        name="Enable Update Limits & Springs",
        default=False,
        description='Enable Limits Update')
    bullet_tool_use_margin: bool(
        name="Collision Margin", default=True, description='Collision Margin')
    bullet_tool_collmargin: float(
        name="Margin",
        default=0.0005,
        min=0.0,
        max=1,
        description="Collision Margin")
    bullet_tool_bounciness: float(
        name="Bounciness",
        default=0.0,
        min=0.0,
        max=10000,
        description="Bounciness")
    bullet_tool_friction: float(
        name="Friction", default=0.5, min=0.0, max=100, description="Friction")
    bullet_tool_iteration: int(name="Iterations", default=60, min=1, max=1000)
    bullet_tool_over_iteration: bool(
        name="Override Iterations",
        default=False,
        description='Override Iterations')
    bullet_tool_breakable: bool(
        name="Breakable",
        default=False,
        description='Enable breakable Constraints')
    bullet_tool_break_threshold: float(
        name="Break Threshold",
        default=10,
        min=0.0,
        max=10000,
        description="Break Threshold. Strength of Object. Break Threshold \
            = Mass * Threshold")
    bullet_tool_absolute_mass: bool(
        name="Absolute",
        default=False,
        description='Break Threshold = Break Threshold')
    bullet_tool_multiplier: bool(
        name="Multiply",
        default=False,
        description='Break Threshold = Break Threshold * Multiplier')
    bullet_tool_search_radius: float(
        name="Search Radius",
        default=3.0,
        min=0.0,
        max=10000,
        description="Near Search radius")

    bullet_tool_neighbours: int(
        name="Neighbour Limit",
        default=3,
        min=1,
        max=60,
        description="Number of Neighbour to Check. More = Slower")

    # for GPencil
    bullet_tool_gpencil_mode: bool(
        name="GPencil Mode",
        default=False,
        description='Disabled =  Edit constraints, Enabled = Edit and \
            Generate Constraints')
    bullet_tool_gpencil_dis: float(
        name="GPencil Distance",
        default=1.0,
        min=0.0,
        max=100,
        description="Distance for GPencil")

    # props for Constraints
    bullet_tool_use_limit_ang_x: bool(name="X Angle", default=False)
    bullet_tool_limit_ang_x_lower: float(
        name="Limit Angle X lower", default=-45.0, min=-360, max=360)
    bullet_tool_limit_ang_x_upper: float(
        name="Limit Angle X upper", default=45.0, min=-360, max=360)

    bullet_tool_use_limit_ang_y: bool(name="Y Angle", default=False)
    bullet_tool_limit_ang_y_lower: float(
        name="Limit Angle Y lower", default=-45.0, min=-360, max=360)
    bullet_tool_limit_ang_y_upper: float(
        name="Limit Angle Y upper", default=45.0, min=-360, max=360)

    bullet_tool_use_limit_ang_z: bool(name="Z Angle", default=False)
    bullet_tool_limit_ang_z_lower: float(
        name="Limit Angle Z lower", default=-45.0, min=-360, max=360)
    bullet_tool_limit_ang_z_upper: float(
        name="Limit Angle Z upper", default=45.0, min=-360, max=360)

    bullet_tool_use_limit_lin_x: bool(name="X Axis", default=False)
    bullet_tool_limit_lin_x_lower: float(
        name="Limit Linear X lower", default=-45.0, min=-10000.0, max=10000)
    bullet_tool_limit_lin_x_upper: float(
        name="Limit Linear X upper", default=45.0, min=-10000.0, max=10000)

    bullet_tool_use_limit_lin_y: bool(name="Y Axis", default=False)
    bullet_tool_limit_lin_y_lower: float(
        name="Limit Linear Y lower", default=-45.0, min=-10000.0, max=10000)
    bullet_tool_limit_lin_y_upper: float(
        name="Limit Linear Y upper", default=45.0, min=-10000.0, max=10000)

    bullet_tool_use_limit_lin_z: bool(name="Z Axis", default=False)
    bullet_tool_limit_lin_z_lower: float(
        name="Limit Linear Z lower", default=-45.0, min=-10000.0, max=10000)
    bullet_tool_limit_lin_z_upper: float(
        name="Limit Linear Z upper", default=45.0, min=-10000.0, max=10000)

    bullet_tool_use_spring_x: bool(name="Use Spring X", default=False)
    bullet_tool_spring_stiffness_x: float(
        name="Stiffness", default=10.0, min=-0.0, max=100)
    bullet_tool_spring_damping_x: float(
        name="Damping", default=0.5, min=-0.0, max=1)

    bullet_tool_use_spring_y: bool(name="Use Spring X", default=False)
    bullet_tool_spring_stiffness_y: float(
        name="Stiffness", default=10.0, min=-0.0, max=100)
    bullet_tool_spring_damping_y: float(
        name="Damping", default=0.5, min=-0.0, max=1)

    bullet_tool_use_spring_z: bool(name="Use Spring X", default=False)
    bullet_tool_spring_stiffness_z: float(
        name="Stiffness", default=10.0, min=-0.0, max=100)
    bullet_tool_spring_damping_z: float(
        name="Damping", default=0.5, min=-0.0, max=1)

    itemlist = [('FIXED', 'Fixed', 'FIXED'),
                ('POINT', 'Point', 'POINT'),
                ('HINGE', 'Hinge', 'HINGE'),
                ('SLIDER', 'Slider', 'SLIDER'),
                ('PISTON', 'Piston', 'PISTON'),
                ('GENERIC', 'Generic', 'GENERIC'),
                ('GENERIC_SPRING', 'Generic Spring', 'GENERIC_SPRING')
                ]

    bullet_tool_Constraint_type: bpy.props.EnumProperty(
        items=itemlist,
        name="Constraint_type")
    # bpy.context.scene['bullet_tool_Constraint_type'] = 0


classes = [BulletToolProps,
           Bullet_Tools,
           OBJECT_OT_MakeConstraints,
           OBJECT_OT_Bullet_X_Connect,
           OBJECT_OT_FromToConstraint,
           OBJECT_OT_Bullet_Update,
           OBJECT_OT_Bullet_Ground_connect,
           OBJECT_OT_Bullet_remove_constraints,
           OBJECT_OT_Bullet_GPencil
           ]


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.WindowManager.bullet_tool = bpy.props.PointerProperty(
        type=BulletToolProps)


def unregister():
    for c in classes:
        bpy.utils.unregister_class(c)
    del bpy.types.WindowManager.bullet_tool


if __name__ == "__main__":
    register()
