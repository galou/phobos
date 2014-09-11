#!/usr/bin/python

"""
Copyright 2014, University of Bremen & DFKI GmbH Robotics Innovation Center

This file is part of Phobos, a Blender Add-On to edit robot models.

Phobos is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License
as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

Phobos is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with Phobos.  If not, see <http://www.gnu.org/licenses/>.

File importer.py

Created on 28 Feb 2014

@author: Kai von Szadkowski
"""

import bpy
import mathutils
import os
import yaml
from collections import namedtuple
import xml.etree.ElementTree as ET
from phobos.utility import *
from . import defs
from . import materials

#This is a really nice pythonic approach to creating a list of constants
Defaults = namedtuple('Defaults', ['mass', 'idtransform'])
defaults = Defaults(0.001, #mass
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #idtransform
                    )

def register():
    print("Registering importer...")

def unregister():
    print("Unregistering importer...")

def cleanUpScene():
    # select all objects
    bpy.ops.object.select_all(action="SELECT")

    # and delete them
    bpy.ops.object.delete()

    # after that we have to clean up all loaded meshes (unfortunately
    # this is not done automatically)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)

    # and all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)

    # and all lights (aka lamps)
    for lamp in bpy.data.lamps:
        bpy.data.lamps.remove(lamp)

def round_float(float_as_str, decimal=6):
    '''
    does not seem to work right for some reason
    '''
    return round(float(float_as_str), decimal)
    
def pos_rot_tree_to_lists(position, rotation):
    '''
    '''
    if position:
        px = round_float(position.find('x').text)
        py = round_float(position.find('y').text)
        pz = round_float(position.find('z').text)
    else:
        px, py, pz = (0, 0, 0)
    if rotation:
        rw = round_float(rotation.find('w').text)
        rx = round_float(rotation.find('x').text)
        ry = round_float(rotation.find('y').text)
        rz = round_float(rotation.find('z').text)
    else:
        rw, rx, ry, rz = (0, 0, 0, 0)
        
    return [px, py, pz], [rw, rx, ry, rz]
        
def calc_pose_formats(position, rotation):
    '''
    test cases with None parameters
    '''
    px, py, pz = position
    if len(rotation) == 3:
        rot = mathutils.Euler(rotation).to_quaternion()
    else:
        rot = rotation
    rw, rx, ry, rz = rot
    pose_dict = {}
    
    translation = [px, py, pz]
    quaternion = mathutils.Quaternion([rw, rx, ry, rz])
    euler = quaternion.to_euler()
    pose_dict['translation'] = translation #mathutils.Matrix.Translation(translation)
    pose_dict['rotation_quaternion'] = [rw, rx, ry, rz]
    pose_dict['rotation_euler'] = [euler.x, euler.y, euler.z]
    rm = quaternion.to_matrix()
    matrix = [[rm[0][0], rm[1][0], rm[2][0], px],
              [rm[0][1], rm[1][1], rm[2][1], py],
              [rm[0][2], rm[1][2], rm[2][2], pz],
              [0,        0,        0,        1]]
    
    pose_dict['matrix'] = matrix #rm.to_4x4()
    
    return pose_dict
    
def add_quaternion(rot1, rot2):
    '''
    Add two rotations in euler angle format.
    '''
    quat1 = mathutils.Quaternion(rot1)
    quat2 = mathutils.Quaternion(rot2)
    quat_sum = quat1 * quat2
    return (quat_sum.w, quat_sum.x, quat_sum.y, quat_sum.z)

class RobotModelParser():
    """Base class for a robot model file parser of a specific type"""

    def __init__(self, filepath):
        self.filepath = filepath
        self.path, self.filename = os.path.split(self.filepath)
        self.robot = {}

    def placeChildLinks(self, parent):
        #print(parent['name']+ ', ', end='')
        children = []
        for l in self.robot['links']:
            if 'parent' in self.robot['links'][l] and self.robot['links'][l]['parent'] == parent['name']:
                children.append(self.robot['links'][l])
        for child in children:
            # 1: set parent relationship (this makes the parent inverse the inverse of the parents world transform)
            parentLink = bpy.data.objects[parent['name']]
            childLink = bpy.data.objects[child['name']]
            bpy.ops.object.select_all(action="DESELECT") #bpy.context.selected_objects = []
            childLink.select = True
            parentLink.select = True
            bpy.context.scene.objects.active = parentLink
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            # 2: move to parents origin by setting the world matrix to the parents world matrix
            childLink.matrix_world = parentLink.matrix_world
            # 3: apply local transform as saved in urdf (change matrix_local from identity to urdf)
            urdf_loc = mathutils.Matrix.Translation(child['pose']['translation'])
            urdf_rot = mathutils.Matrix(child['pose']['matrix']).to_4x4()
            urdfmatrix = urdf_loc * urdf_rot
            childLink.matrix_local = urdfmatrix
            # 4: be happy, as world and basis are now the same and local is the transform to be exported to urdf
            # 5: take care of the rest of the tree
            self.placeChildLinks(child)

    def placeLinkSubelements(self, link):
        #urdf_sca = #TODO: solve problem with scale
        # 3.2: make sure to take into account visual information #TODO: also take into account inertial and joint axis (for joint sphere) and collision (bounding box)
        #* urdf_visual_loc * urdf_visual_rot #*urdf_sca
        parentLink = bpy.data.objects[link['name']]
        if 'inertial' in link:
            if 'pose' in link['inertial']:
                urdf_geom_loc = mathutils.Matrix.Translation(link['inertial']['pose']['translation'])
                urdf_geom_rot = mathutils.Matrix(link['inertial']['pose']['matrix']).to_4x4()
            else:
                urdf_geom_loc = mathutils.Matrix.Identity(4)
                urdf_geom_rot = mathutils.Matrix.Identity(4)
            #print(link['name'], link['inertial'])
            #print(link['inertial']['name'])
            geoname = link['inertial']['name'] #g
            geom = bpy.data.objects[geoname]
            bpy.ops.object.select_all(action="DESELECT")
            geom.select = True
            parentLink.select = True
            bpy.context.scene.objects.active = parentLink
            bpy.ops.object.parent_set(type='BONE_RELATIVE')
            #geom.matrix_world = parentLink.matrix_world #FIXME: this applies the scale of the parent, making boxes BIIIG
            geom.matrix_local = urdf_geom_loc * urdf_geom_rot
        stream = open(self.filepath+'current_link.yml', 'w')
        yaml.dump(link, stream)
        stream.close()
        for geomsrc in ['visual', 'collision']:
            if geomsrc in link:
                for g in link[geomsrc]:
                    geom = link[geomsrc][g]
                    #print([key for key in geom])
                    if 'pose' in geom:
                        #print('geom:', geom)
                        urdf_geom_loc = mathutils.Matrix.Translation(geom['pose']['translation'])
                        urdf_geom_rot = mathutils.Matrix(geom['pose']['matrix']).to_4x4()
                    else:
                        urdf_geom_loc = mathutils.Matrix.Identity(4)
                        urdf_geom_rot = mathutils.Matrix.Identity(4)
                    geoname = geom['name'] #g
                    geom = bpy.data.objects[geoname]
                    bpy.ops.object.select_all(action="DESELECT")
                    geom.select = True
                    parentLink.select = True
                    bpy.context.scene.objects.active = parentLink
                    bpy.ops.object.parent_set(type='BONE_RELATIVE')
                    geom.matrix_world = parentLink.matrix_world
                    geom.matrix_local = urdf_geom_loc * urdf_geom_rot

    def createGeometry(self, viscol, geomsrc):
        newgeom = None
        if viscol['geometry'] is not {}:
            bpy.ops.object.select_all(action='DESELECT')
            geom = viscol['geometry']
            geomtype = geom['geometryType']
            # create the Blender object
            # tag all objects
            for obj in bpy.data.objects:
                obj.tag = True
            if geomtype == 'mesh':
                filetype = geom['filename'].split('.')[-1]
                print(geom['filename'])
                if filetype == 'obj' or filetype == 'OBJ':
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, geom['filename']))
                elif filetype == 'stl' or filetype == 'STL':
                    bpy.ops.import_mesh.stl(filepath=os.path.join(self.path, geom['filename']))
                # hack for test:
                elif filetype == 'bobj' or filetype == 'BOBJ':
                    filename = 'visual_' + geom['filename'].split('.')[0] + '.obj'
                    bpy.ops.import_scene.obj(filepath=os.path.join(self.path, filename))
                else:
                    print('ERROR: Could not import object.')
                # find the newly imported obj
                print('new object name:', viscol['name'])
                print("existing objects' names:")
                for obj in bpy.data.objects:
                    print(obj.name)
                    if not obj.tag:
                        newgeom = obj
                        #with obj file import, blender only turns the object, not the vertices,
                        #leaving a rotation in the matrix_basis, which we here get rid of
                        if filetype == 'obj':
                            bpy.ops.object.select_all(action='DESELECT')
                            newgeom.select = True
                            bpy.ops.object.transform_apply(rotation=True)
                newgeom.name = viscol['name']
            elif geomtype == 'box':
                newgeom = createPrimitive(viscol['name'],
                                          geomtype,
                                          geom['size'],
                                          defs.layerTypes[geomsrc]
                                          )
            elif geomtype == 'cylinder':
                newgeom = createPrimitive(viscol['name'],
                                          geomtype,
                                          (geom['radius'], geom['length']),
                                          defs.layerTypes[geomsrc]
                                          )
            elif geomtype == 'sphere':
                newgeom = createPrimitive(viscol['name'],
                                          geomtype,
                                          geom['radius'], #tuple would cause problem here
                                          defs.layerTypes[geomsrc]
                                          )
            else:
                print("### ERROR: Could not determine geometry type of " + geomsrc + viscol['name'] + '. Placing empty coordinate system.')
            if newgeom is not None:
                newgeom.MARStype = geomsrc
                newgeom.select = True
                if 'scale' in geom:
                    newgeom.scale = geom['scale']
                bpy.ops.object.transform_apply(scale=True)
                newgeom['geometryType'] = geomtype
                #TODO: which other properties remain?
            #FIXME: place empty coordinate system and return...what?
        return newgeom

    def createInertial(self, name, inertial):
        bpy.ops.object.select_all(action='DESELECT')
        inert = createPrimitive('inertial_'+name, 'sphere', 0.01, 0, 'None', (0, 0, 0))
        inert.select = True
        bpy.ops.object.transform_apply(scale=True)
        #obj = bpy.context.object
        #obj.name = name
        for prop in inertial:
            if prop not in ['pose'] and inertial[prop] is not None:
                inert[prop] = inertial[prop]
        inert.MARStype = 'inertial'
        return inert

    def createLink(self, link):
        print("Creating link", link['name'])
        #create base object ( =armature)
        bpy.ops.object.select_all(action='DESELECT')
        #bpy.ops.view3d.snap_cursor_to_center()
        bpy.ops.object.armature_add(layers=defLayers([0]))
        newlink = bpy.context.active_object #print(bpy.context.object) #print(bpy.context.scene.objects.active) #bpy.context.selected_objects[0]
        newlink.name = link['name']
        newlink.location = (0.0, 0.0, 0.0)
        newlink.scale = (0.3, 0.3, 0.3) #TODO: make this depend on the largest visual or collision object
        #newlink.scale = (1, 0.1, 0.01)
        bpy.ops.object.transform_apply(scale=True)
        newlink.MARStype = 'link'
        if newlink.name != link['name']:
            print("Warning, name conflict!")
        # place inertial
        if 'inertial' in link:
            self.createInertial(link['name'], link['inertial'])
        # place visual
        if 'visual' in link:
            for v in link['visual']:
                visual = link['visual'][v]
                if 'geometry' in visual:
                    self.createGeometry(visual, 'visual')
        # place collision
        if 'collision' in link:
            for c in link['collision']:
                collision = link['collision'][c]
                if 'geometry' in collision:
                    self.createGeometry(collision, 'collision')
        return newlink

    def createBlenderModel(self): #TODO: solve problem with duplicated links (linklist...namespaced via robotname?)
        """Creates the blender object representation of the imported model."""
        print("\n\nCreating Blender model...")
        for l in self.robot['links']:
            #print(l + ', ', end='')
            link = self.robot['links'][l]
            #print(link['name'])
            self.createLink(link)

        #build tree recursively and correct translation & rotation on the fly
        for l in self.robot['links']:
            if not 'parent' in self.robot['links'][l]:
                root = self.robot['links'][l]
        print("\n\nPlacing links...")
        self.placeChildLinks(root)
        for link in self.robot['links']:
            self.placeLinkSubelements(self.robot['links'][link])



class MARSModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a MARS scene"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)
        
        self.xml_tree = None
        self.link_index_dict = {}
        self.link_groups = {}
        self.material_indices = {}
        self.link_indices = set([])
        self.vis_coll_groups = {}
        self.base_poses = {}

    def parseModel(self):
        '''
        '''
        print("\nParsing MARS scene from", self.filepath)
        self.tree = ET.parse(self.filepath)
        root = self.tree.getroot()
        
        # robot name is not included in MARS scenes, right?
        
        nodes = root.find('nodelist')
        joints = root.find('jointlist')
        sensors = root.find('sensorlist')
        motors = root.find('motorlist')
        controllers = root.find('controllerlist')
        materials = root.find('materiallist')
        
        self._apply_relative_ids(nodes)
        self._get_links(nodes, joints)
        self._parse_materials(materials)
        
        self.robot['links'] = self._parse_links(nodes)
        self.robot['joints'] = self._parse_joints(joints)
        self.robot['sensors'] = self._parse_sensors(sensors)
        #self.robot['motors'] = self._parse_motors(motors)
        #self.robot['controllers'] = self._parse_controllers(controllers)
        self.robot['groups'] = self.link_groups
        self._parse_additional_visuals_and_collisions(self.robot, nodes)
        
        with open(self.filepath+'_debug.yml', 'w') as outputfile:
            yaml.dump(self.robot, outputfile)
            
        print(self.robot)
    
    def _get_links(self, nodes, joints):
        '''
        '''
        for joint in joints:
            parent = int(joint.find('nodeindex1').text)
            self.link_indices.update([parent])
            child = int(joint.find('nodeindex2').text)
            self.link_indices.update([child])
        
    def _parse_materials(self, materials_tree):
        '''
        Parse the materials from the MARS scene.
        
        TODO: change 'materials' so that superfluous 'color' entry is not needed
        '''
        material_list = []
        for material in materials_tree:
            material_dict = {}
            mat_id = int(material.find('id').text)
            
            # check needs to be explicit for future versions
            if material.find('name') is not None:
                name = material.find('name').text
            else:
                name = 'material_' + str(mat_id)
            material_dict['name'] = name
            
            for xml_colour in defs.MARSrevlegdict:
                colour = material.find(xml_colour)
                if colour is not None:
                    r = round_float(colour.find('r').text)
                    g = round_float(colour.find('g').text)
                    b = round_float(colour.find('b').text)
                    a = round_float(colour.find('a').text)
                    py_colour = defs.MARSrevlegdict[xml_colour]
                    material_dict[py_colour] = [r, g, b, a]
                    # for now:
                    material_dict['color'] = [r, g, b, a]
            
            
            transparency = material.find('transparency')
            if transparency is not None:
                material_dict['transparency'] = round_float(transparency.text)
            else:
                material_dict['transparency'] = 0.0
                
            material_dict['shininess'] = round_float(material.find('shininess').text)
                
            self.material_indices[mat_id] = material_dict
            
        # solution for now; there is a better solution plus this is duplicate code:
            material_list.append(material_dict)
            
        for m in material_list:
            materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1])
    
    def _parse_geometry(self, node, mode):
        '''
        mesh incomplete
        '''
        size = None
        if mode == 'visual':
            size = node.find('visualsize')
            mesh_file = node.find('filename')
            if mesh_file is not None:
                geometry_type = 'mesh'
            else:
                geometry_type = node.find('physicmode').text
        elif mode == 'collision':
            size = node.find('extend')
            geometry_type = node.find('physicmode').text
        
        geometry_dict = {}
        geometry_dict['geometryType'] = geometry_type
        
        if geometry_type == 'box' or geometry_type == 'mesh':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            z = round_float(size.find('z').text)
            geometry_dict['size'] = [x, y, z]
            if geometry_type == 'mesh':
                filename = node.find('filename').text
                geometry_dict['filename'] = filename
        elif geometry_type == 'sphere' or geometry_type == 'cylinder' or geometry_type == 'capsule':
            radius = round_float(size.find('x').text) / 2.0
            geometry_dict['radius'] = radius
            if geometry_type == 'cylinder' or geometry_type == 'capsule':
                height = round_float(size.find('z').text)
                geometry_dict['height'] = height
        elif geometry_type == 'plane':
            x = round_float(size.find('x').text)
            y = round_float(size.find('y').text)
            geometry_dict['size'] = [x, y]
        return geometry_dict
    
    def _parse_visual(self, visuals_dict, node):
        '''
        '''
        visual_dict = {}
        name = 'visual_' + node.get('name')
        visual_dict['name'] = name
        index = int(node.find('index').text)
        
        visual_position = node.find('visualposition')
        if visual_position is None:
            visual_position = node.find('position')
        visual_rotation = node.find('visualrotation')
        if visual_rotation is None:
            visual_rotation = node.find('rotation')
        position, rotation = pos_rot_tree_to_lists(visual_position, visual_rotation)
        #visual_dict['pose'] = calc_pose_formats(position, rotation)
        
        visual_dict['pose'] = self.base_poses[index]['pose']
        
        mat_index = int(node.find('material_id').text)
        visual_dict['material'] = self.material_indices[mat_index]
        
        geometry_dict = self._parse_geometry(node, 'visual')
        visual_dict['geometry'] = geometry_dict
        
        visuals_dict[name] = visual_dict
        
    def _parse_collision(self, collisions_dict, node):
        '''
        '''
        collision_dict = {}
        name = 'collision_' + node.get('name')
        collision_dict['name'] = name
        index = int(node.find('index').text)
        #print('base_poses:', self.base_poses)
        #print('collision_pose:', self.base_poses[index])
        
        
        collision_position = node.find('position')
        collision_rotation = node.find('rotation')
        position, rotation = pos_rot_tree_to_lists(collision_position, collision_rotation)
        collision_dict['pose'] = calc_pose_formats(position, rotation)
        
        #collision_dict['pose'] = self.base_poses[index]['pose']
        
        bitmask = int(float(node.find('coll_bitmask').text))
        collision_dict['bitmask'] = bitmask
        
        geometry_dict = self._parse_geometry(node, 'collision')
        collision_dict['geometry'] = geometry_dict
        
        max_contacts = node.find('cmax_num_contacts')
        if max_contacts is not None:
            collision_dict['max_contacts'] = int(max_contacts.text)
        
        collisions_dict[name] = collision_dict
        
    def _parse_inertial(self, link_dict, node):
        '''
        pose in inertia does not seem to be available
        '''
        inertial_dict = {}
        
        mass = node.find('mass')
        if mass is not None:
            inertial_dict['mass'] = round_float(mass.text)
        
        inertia = node.find('inertia')
        ## if no inertia provided use identity matrix
        #if inertia is None or not bool(inertia.text):
        #    inertial_dict['inertia'] = [1.0, 0.0, 0.0,
        #                                     1.0, 0.0,
        #                                          1.0]
        if inertia is not None and bool(inertia.text):  # 'inertia' states whether the defined inertia values are to be used
            i00 = round_float(node.find('i00').text)
            i01 = round_float(node.find('i01').text)
            i02 = round_float(node.find('i02').text)
            i11 = round_float(node.find('i11').text)
            i12 = round_float(node.find('i12').text)
            i22 = round_float(node.find('i22').text)
            inertial_dict['inertia'] = [i00, i01, i02,
                                             i11, i12,
                                                  i22]
                                
        if inertial_dict is not {}:
            position, rotation = pos_rot_tree_to_lists(None, None)
            inertial_dict['pose'] = calc_pose_formats(position, rotation)
            inertial_dict['name'] = 'inertial_' + node.get('name')
            link_dict['inertial'] = inertial_dict
        
    
    def _parse_links(self, nodes):
        '''
        '''
        links_dict = {}
        for node in nodes:
            index = int(node.find('index').text)
            name = node.get('name')
            self.link_index_dict[index] = name
            
            group = int(node.find('groupid').text)
            node_group_dict = {'name': name,
                               'index': index}
            if group in self.link_groups:
                self.link_groups[group].append(node_group_dict)
            else:
                self.link_groups[group] = [node_group_dict]
                
            if index in self.link_indices:
                link_dict = {}
                
                #link_dict['filename'] = node.find('filename').text
                link_dict['name'] = node.attrib['name']
                
                pose = self.base_poses[index]['pose']
                link_dict['pose'] = pose
                
                visuals_dict = {}
                self._parse_visual(visuals_dict, node)
                link_dict['visual'] = visuals_dict
                
                collisions_dict = {}
                self._parse_collision(collisions_dict, node)
                link_dict['collision'] = collisions_dict
                
                inertial_dict = self._parse_inertial(link_dict, node)
                
                links_dict[name] = link_dict
                
            else:
                self.vis_coll_groups[index] = group
                
        return links_dict
        
    def _parse_additional_visuals_and_collisions(self, model, nodes):
        '''
        not well-tested yet
        '''
        for node in nodes:
            index = int(node.find('index').text)
            if index in self.vis_coll_groups:
                group = self.link_groups[self.vis_coll_groups[index]]
                for group_node in group:
                    if group_node['index'] in self.link_indices:
                        visuals_dict = model['links'][group_node['name']]['visual']
                        self._parse_visual(visuals_dict, node)
                        model['links'][group_node['name']]['visual'] = visuals_dict
                        
                        collisions_dict = model['links'][group_node['name']]['collision']
                        self._parse_collision(collisions_dict, node)
                        model['links'][group_node['name']]['collision'] = collisions_dict
                        
                        break
        
    def _parse_joints(self, joints):
        '''
        '''
        joints_dict = {}
        for joint in joints:
            joint_dict = {}
            name = joint.get('name')
            joint_dict['name'] = name
            joint_dict['type'] = joint.find('type').text
            
            parent_index = int(joint.find('nodeindex1').text)
            joint_dict['parent'] = self.link_index_dict[parent_index]
            child_index = int(joint.find('nodeindex2').text)
            joint_dict['child'] = self.link_index_dict[child_index]
            
            joints_dict[name] = joint_dict
        return joints_dict
        
    def _apply_relative_ids(self, nodes):
        '''
        seems to work but not well-tested yet
        '''
        base_nodes = {}     # poses with references already applied plus raw pos and rot values for easier calculations
        rel_nodes = {}
        for node in nodes:
            index = int(node.find('index').text)
            xml_position = node.find('position')
            xml_rotation = node.find('rotation')
            position, rotation = pos_rot_tree_to_lists(xml_position, xml_rotation)
            #print('position:', position)
            #print('rotation:', rotation)
            pose = calc_pose_formats(position, rotation)
            if node.find('relativeid') is not None:
                rel_node_dict = {'pose': pose,
                             'rel_id': int(node.find('relativeid').text),
                             'raw_pos': position,
                             'raw_rot': rotation}
                rel_nodes[index] = rel_node_dict
            else:
                base_node_dict = {'pose': pose,
                                  'raw_pos': position,
                                  'raw_rot': rotation}
                base_nodes[index] = base_node_dict
        num_rel_nodes = -1
        while rel_nodes:        
            to_delete = []
            if len(rel_nodes) == num_rel_nodes:             # check for infinite loop (happens if referenced node does not exist)
                print('Error: non-existant relative id')
                break
            num_rel_nodes = len(rel_nodes)
            for rel_index in rel_nodes:
                rel_node = rel_nodes[rel_index]
                base = rel_node['rel_id']
                if base in base_nodes:
                    base_pose = base_nodes[base]['pose']
                    rel_pose = rel_node['pose']
                    applied_pos = [bp+rp for bp, rp in zip(base_nodes[base]['raw_pos'], rel_node['raw_pos'])]
                    applied_rot = add_quaternion(base_nodes[base]['raw_rot'], rel_node['raw_rot'])
                    #applied_rot = [br+rr for br, rr in zip(base_nodes[base]['raw_rot'], rel_node['raw_rot'])]
                    #applied_rot = rel_node['raw_rot']
                    base_nodes[rel_index] = {'pose': calc_pose_formats(applied_pos, applied_rot),
                                             'raw_pos': applied_pos,
                                             'raw_rot': applied_rot}
                    to_delete.append(rel_index)
            for index in to_delete:
                del rel_nodes[index]
        self.base_poses = base_nodes
    
    def _parse_sensors(self, sensors):
        '''
        'link' is missing in manual
        '''
        sensors_dict = {}
        for sensor in sensors:
            sensor_dict = {}
            name = sensor.get('name')
            sensor_dict['link'] = None  # where to get this?
            sensor_dict['sensorType'] = sensor.get('type')
            sensors_dict[name] = sensor_dict
        return sensors_dict
    
    def _parse_motors(self, motors):
        '''
        don't know yet what motors look like
        '''
        pass
    
    def _parse_controllers(self, controllers):
        '''
        don't know yet what controllers look like
        '''
        pass


class URDFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a URDF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        print("\nParsing URDF model from", self.filepath)
        self.tree = ET.parse(self.filepath)
        self.root = self.tree.getroot()#[0]
        self.robot["name"] = self.root.attrib["name"]
        if 'version' in self.root.attrib:
            self.robot["version"] = self.root.attrib['version'] #TODO: implement version functionality (time code)

        #write links to dictionary
        links = {}
        print("\n\nParsing links..")
        for link in self.root.iter('link'):
            newlink = self.parseLink(link)
            #write link to list
            links[newlink['name']] = newlink
            #print(newlink)
        self.robot['links'] = links

        #write joints to dictionary
        joints = {}
        print("\n\nParsing joints..")
        for joint in self.root.iter('joint'):
            if joint.find('parent') is not None: #this is needed as there are "joint" tags e.g. in transmission
                newjoint, pose = self.parseJoint(joint)
                self.robot['links'][newjoint['child']]['pose'] = pose
                joints[newjoint['name']] = newjoint
                #print(newjoint)
        self.robot['joints'] = joints

        #find any links that still have no pose (most likely because they had no parent)
        for link in links:
            if not 'pose' in links[link]:
                position, rotation = pos_rot_tree_to_lists(None, None)
                links[link]['pose'] = calc_pose_formats(position, rotation)
            #print(link, links[link]['pose'])

        #write parent-child information to nodes
        print("\n\nWriting parent-child information to nodes..")
        for j in self.robot['joints']:
            joint = self.robot['joints'][j]
            self.robot['links'][joint['child']]['parent'] = joint['parent']
            #print(joint['parent'] + ', ', end='')

        #now some debug output
        with open(self.filepath+'_debug.yml', 'w') as outputfile:
            outputfile.write(yaml.dump(self.robot))#, default_flow_style=False)) #last parameter prevents inline formatting for lists and dictionaries

<<<<<<< HEAD
        material_list = [] #TODO: build dictionary entry for materials
=======
        materiallist = [] #TODO: build dictionary entry for materials
>>>>>>> 12646ec5fb1754ff0d399835534ae72341373158
        print("\n\nParsing materials..")
        for material in self.root.iter('material'):
            newmaterial = {a: material.attrib[a] for a in material.attrib}
            color = material.find('color')
            if color is not None:
                #print(material.attrib['name'] + ', ', end='')
                newmaterial['color'] = parse_text(color.attrib['rgba'])
<<<<<<< HEAD
                material_list.append(newmaterial)
        for m in material_list:
=======
                materiallist.append(newmaterial)
        for m in materiallist:
>>>>>>> 12646ec5fb1754ff0d399835534ae72341373158
            materials.makeMaterial(m['name'], tuple(m['color'][0:3]), (1, 1, 1), m['color'][-1]) #TODO: handle duplicate names? urdf_robotname_xxx?

    def parseLink(self, link):
        novisual = True
        #print(link.attrib['name'] + ', ', end='')
        newlink = {a: link.attrib[a] for a in link.attrib}

        #parse 'inertial'
        inertial = link.find('inertial')
        if inertial is not None: # !!! 'if Element' yields none if the Element contains no children, thus this notation !!!
            newlink['inertial'] = {}
            origin = inertial.find('origin')
            if origin is not None:
                raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                newlink['inertial']['pose'] = calc_pose_formats(raw_pose[:3], raw_pose[3:])
            else:
                position, rotation = pos_rot_tree_to_lists(None, None)
                newlink['inertial']['pose'] = calc_pose_formats(position, rotation)
            mass = inertial.find('mass')
            if mass is not None:
                newlink['inertial']['mass'] = float(mass.attrib['value'])
            inertia = inertial.find('inertia')
            if inertia is not None:
                values = []
                newlink['inertial']['inertia'] = values.append(inertia.attrib[a] for a in inertia.attrib)
            newlink['inertial']['name'] = 'inertial_' + newlink['name']

        #parse 'visual'
        newlink['visual'] = {}
        i=0
        for visual in link.iter('visual'):
            try:
                visname = visual.attrib['name']
            except KeyError:
                visname = 'visual_' + str(i) + '_' + newlink['name']
                i += 1
            newlink['visual'][visname] = {a: visual.attrib[a] for a in visual.attrib}
            vis = newlink['visual'][visname]
            vis['name'] = visname
            origin = visual.find('origin')
            if origin is not None:
                 raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                 vis['pose'] = calc_pose_formats(raw_pose[:3], raw_pose[3:])
            else:
                vis['pose'] = calc_pose_formats(None, None)
            geometry = visual.find('geometry')
            if geometry is not None:
                vis['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                vis['geometry']['geometryType'] = geometry[0].tag
                novisual = False
                if geometry[0].tag == 'mesh':
                    vis['geometry']['filename'] = geometry[0].attrib['filename'] #TODO: remove this, also from export, as it is double
            else: #if geometry is None
                print("\n### WARNING: No geometry information for visual element, trying to parse from collision data.")
            material = visual.find('material')
            if material is not None:
                vis['material'] = {'name': material.attrib['name']}
                color = material.find('color')
                if color is not None:
                    vis['material']['color'] = parse_text(color.attrib['rgba'])
            else:
                vis['material'] = {'name':'None'} #TODO: this is a hack!
                print("\n### Warning: No material provided for link", newlink['name'])
        if newlink['visual'] == {}: #'else' cannot be used as we don't use a break
            del(newlink['visual'])
            print("\n### WARNING: No visual information provided for link", newlink['name'])

        #parse 'collision'
        newlink['collision'] = {}
        i=0
        for collision in link.iter('collision'):
            try:
                colname = collision.attrib['name']
            except KeyError:
                colname = 'collision_' + str(i) + '_' + newlink['name']
                i += 1
            newlink['collision'][colname] = {a: collision.attrib[a] for a in collision.attrib}
            col = newlink['collision'][colname]
            col['name'] = colname
            origin = collision.find('origin')
            if origin is not None:
                raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
                col['pose'] = calc_pose_formats(raw_pose[:3], raw_pose[3:])
            else:
                col['pose'] = calc_pose_formats(None, None)
            geometry = collision.find('geometry')
            if geometry is not None:
                col['geometry'] = {a: parse_text(geometry[0].attrib[a]) for a in geometry[0].attrib}
                col['geometry']['geometryType'] = geometry[0].tag
                if geometry[0].tag == 'mesh':
                    col['geometry']['filename'] = geometry[0].attrib['filename']
                #if novisual:
                #    newlink['visual']['geometry'] = col['geometry']
            else:
                print("\n### WARNING: No collision geometry information provided for link", newlink['name'] + '.')
                if novisual:
                    print("\n### WARNING:", newlink['name'], "is empty.")
        if newlink['collision'] == {}: #'else' cannot be used as we don't use a break
            del(newlink['collision'])
            print("\n### WARNING: No collision information provided for link", newlink['name'] + '.')
            if novisual:
                print("\n### WARNING:", newlink['name'], "is empty.")
        return newlink

    def parseJoint(self, joint):
        #print(joint.attrib['name']+', ', end='')
        newjoint = {a: joint.attrib[a] for a in joint.attrib}
        try:
            origin = joint.find('origin')
            origindict = {'xyz': origin.attrib['xyz'].split(), 'rpy': origin.attrib['rpy'].split()}
        except AttributeError:
            origindict = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
        newjoint['parent'] = joint.find('parent').attrib['link']
        newjoint['child'] = joint.find('child').attrib['link']
<<<<<<< HEAD
        raw_pose = [float(num) for num in (origin.attrib['xyz'].split() + origin.attrib['rpy'].split())]
        pose = calc_pose_formats(raw_pose[:3], raw_pose[3:])
=======
        pose = [float(num) for num in (origindict['xyz'] + origindict['rpy'])]
>>>>>>> 12646ec5fb1754ff0d399835534ae72341373158
        #axis
        #calibration
        #dynamics
        #limit
        #mimic
        #safety_controller
        return newjoint, pose

class SMURFModelParser(RobotModelParser):
    """Class derived from RobotModelParser which parses a SMURF model"""

    def __init__(self, filepath):
        RobotModelParser.__init__(self, filepath)

    def parseModel(self):
        print("Parsing SMURF model...")
        stream = open(self.filepath, 'r')
        smurf_spec = yaml.load(stream)
        smurf_files = smurf_spec['files']
        for smurf_file in smurf_files:
            if smurf_file.split('.')[-1] == 'urdf':
                urdf_parser = URDFModelParser(os.path.dirname(self.filepath) + '/' + smurf_file)
                urdf_parser.parseModel()
                self.robot = urdf_parser.robot
            elif smurf_file.split('.')[-1] in ['yml', 'yaml']:
                pass
                # TODO


class RobotModelImporter(bpy.types.Operator):
    """Importer for MARS-compatible model or scene files"""
    bl_idname = "obj.import_robot_model"
    bl_label = "Import robot model file from various formats"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'FILE'

    # creating property for storing the path to the .scn file
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    # set a filter to only consider .scn files (only used internally)
    #filter_glob = bpy.props.StringProperty(default="*.*",options={'HIDDEN'})

    @classmethod
    def poll(cls, context):
        return context is not None

    def execute(self, context):
        # get the chosen file path
        #directory, filename = os.path.split(self.filepath)
        modeltype = self.filepath.split('.')[-1]

        if modeltype == 'scene':
            importer = MARSModelParser(self.filepath)
        elif modeltype == 'urdf':
            importer = URDFModelParser(self.filepath)
        elif modeltype == 'smurf' or modeltype == 'yml' or modeltype == 'yaml':
            importer = SMURFModelParser(self.filepath)
        else:
            print("Unknown model format, aborting import...")

        importer.parseModel()
        importer.createBlenderModel()

        return {'FINISHED'}

    def invoke(self, context, event):
        # create the open file dialog
        context.window_manager.fileselect_add(self)

        return {'RUNNING_MODAL'}

# Register and add to the file selector
bpy.utils.register_class(RobotModelImporter)


def main():
    # call the newly registered operator
    cleanUpScene()
    bpy.ops.import_robot_model('INVOKE_DEFAULT')

if __name__ == '__main__':
    main()
