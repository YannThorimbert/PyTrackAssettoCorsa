import bpy
import os

# Function to delete an object if it exists
def delete_object(obj_name):
    obj = bpy.data.objects.get(obj_name)
    if obj is not None:
        # Deselect all objects
        bpy.ops.object.select_all(action='DESELECT')
        # Select the object to delete
        obj.select_set(True)
        # Delete the object
        bpy.ops.object.delete()

# Function to import an STL file if it exists
def import_stl(file_path):
    if os.path.exists(file_path):
        bpy.ops.import_mesh.stl(filepath=file_path)
    else:
        print(f"STL file '{file_path}' does not exist")

def set_name_material_collection(obj, obj_name, material_name, collection_name):
    obj.name = obj_name
    # Check if the material already exists
    mat = bpy.data.materials.get(material_name)
    # if mat is None:
    #     mat = bpy.data.materials.new(name=material_name) #create new mat
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)
    # Remove the object from all collections
    for collection in obj.users_collection:
        collection.objects.unlink(obj)
    # Check if the collection exists
    collection = bpy.data.collections.get(collection_name)
    if collection is None:
        # If the collection does not exist, create a new one
        collection = bpy.data.collections.new(name=collection_name)
        bpy.context.scene.collection.children.link(collection)
    # Link the object to the specified collection
    collection.objects.link(obj)

def apply_steps(object_name, material_name, collection_name):
    # delete_object(object_name)
    import_stl(f"{STL_ROOT}/{object_name}.stl")
    obj = bpy.context.selected_objects[0] if bpy.context.selected_objects else None
    if obj is not None:
        set_name_material_collection(obj, object_name, material_name, collection_name)
    else:
        print(f"Object named '{object_name}' not found")

def save_as_fbx(filepath):
    # Assurer que Blender est en mode 'OBJECT' pour l'exportation
    if bpy.context.active_object is not None:
        if bpy.context.active_object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

    try:
        bpy.ops.export_scene.fbx(
            filepath=filepath,
            use_selection=False,
            axis_forward='-Z',
            axis_up='Y',
            apply_unit_scale=True,
            global_scale=1.0,
            apply_scale_options='FBX_SCALE_UNITS'
        )
        print(f"Project saved as FBX at '{filepath}'")
    except Exception as e:
        print(f"Failed to save FBX: {e}")

def select_longest_edge(obj):
    mesh = obj.data
    longest_edge = None
    max_length = 0.0
    # Calculer la longueur de chaque arête
    for edge in mesh.edges:
        vert1 = mesh.vertices[edge.vertices[0]].co
        vert2 = mesh.vertices[edge.vertices[1]].co
        length = (vert1 - vert2).length
        if length > max_length:
            max_length = length
            longest_edge = edge
    # Sélectionner l'arête la plus longue
    if longest_edge is not None:
        longest_edge.select = True

def fill_inside_mesh(obj_name, new_obj_name):
    # Assurer que l'objet existe et est un mesh
    obj = bpy.data.objects.get(obj_name)
    if obj and obj.type == 'MESH':
        objects_before = set(bpy.data.objects)
        # Passer en mode édition
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        # Désélectionner tout
        bpy.ops.mesh.select_all(action='DESELECT')
        # Passer en mode de sélection par arêtes
        bpy.context.tool_settings.mesh_select_mode = (False, True, False)
        # Mettre à jour la scène
        bpy.ops.object.mode_set(mode='OBJECT')
        # Sélectionner la première arête
        # obj.data.edges[edge_number].select = True
        select_longest_edge(obj)
        # Retourner en mode édition
        bpy.ops.object.mode_set(mode='EDIT')
        # Sélectionner la boucle d'arêtes
        bpy.ops.mesh.loop_multi_select(ring=False)
        # Remplir la face
        bpy.ops.mesh.edge_face_add()
        # Séparer par sélection
        bpy.ops.mesh.separate(type='SELECTED')
        # Retourner en mode objet
        bpy.ops.object.mode_set(mode='OBJECT')
        # Comparer les objets avant et après pour trouver le nouvel objet
        objects_after = set(bpy.data.objects)
        new_object = (objects_after - objects_before).pop()  # Prendre le nouvel objet ajouté
        set_name_material_collection(new_object, new_obj_name, "Grass", "Track")
    else:
        print("L'objet spécifié n'existe pas ou n'est pas un mesh")
    
def apply_cube_projection_uv_scaling(obj_name, scale_factor):
    # Trouver l'objet par son nom
    obj = bpy.data.objects.get(obj_name)
    if not obj or obj.type != 'MESH':
        print(f"L'objet {obj_name} n'existe pas ou n'est pas un mesh.")
        return
    # Sélectionner l'objet
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    # Passer en mode édition
    bpy.ops.object.mode_set(mode='EDIT')
    # Sélectionner tout pour le UV mapping
    bpy.ops.mesh.select_all(action='SELECT')
    # Appliquer la projection cubique
    bpy.ops.uv.cube_project(scale_to_bounds=False)
    # Passer en mode de modification des UVs pour ajuster l'échelle
    bpy.ops.object.mode_set(mode='OBJECT')
    uv_layer = obj.data.uv_layers.active.data
    # Ajuster l'échelle des UVs
    for poly in obj.data.polygons:
        for loop_index in poly.loop_indices:
            uv_layer[loop_index].uv *= scale_factor
    # Retourner en mode objet
    bpy.ops.object.mode_set(mode='OBJECT')
    print(f"UV mapping et scaling appliqués à l'objet {obj_name}.")


##############################################################################
try:
    bpy.ops.object.mode_set(mode='OBJECT')
except:
    pass
STL_ROOT = "C:/Users/Admin/Documents/programmation/PyTrackAssettoCorsa"
potentially_to_delete = ["1ROAD1",
                         "1GRASS_L_LAND", "1GRASS_R_LAND", "1GRASS_INSIDE", "1ROAD_L", "1ROAD_R",
                         "1KERB_L1", "1KERB_R1", "1KERB_L2", "1KERB_R2"]
for name in potentially_to_delete:
    delete_object(name)
apply_steps("1ROAD1", "Asphalt", "Track")
apply_steps("1GRASS_L_LAND", "Grass", "Track")
apply_steps("1GRASS_R_LAND", "Grass", "Track")
apply_steps("1ROAD_L", "Asphalt", "Track") #smooth, external grass
apply_steps("1ROAD_R", "Asphalt", "Track") #smooth, external grass
apply_steps("1KERB_L1", "Kerb1", "Kerb")
apply_steps("1KERB_L2", "Kerb2", "Kerb")
apply_steps("1KERB_R1", "Kerb1", "Kerb")
apply_steps("1KERB_R2", "Kerb2", "Kerb")
fill_inside_mesh("1GRASS_R_LAND", "1GRASS_INSIDE") #creates 1GRASS_INSIDE

apply_cube_projection_uv_scaling("1ROAD1", 180.)
apply_cube_projection_uv_scaling("1ROAD_L", 180.)
apply_cube_projection_uv_scaling("1ROAD_R", 180.)

apply_cube_projection_uv_scaling("1GRASS_L_LAND", 180.)
apply_cube_projection_uv_scaling("1GRASS_R_LAND", 180.)
apply_cube_projection_uv_scaling("1GRASS_INSIDE", 180.)


# save_as_fbx("./my_track.fbx")