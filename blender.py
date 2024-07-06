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
        obj.data.edges[0].select = True
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
    


#bpy.ops.object.mode_set(mode='OBJECT')

STL_ROOT = "C:/Users/Admin/Documents/programmation/PyTrackAssettoCorsa"
potentially_to_delete = ["1ROAD1", "1GRASS0", "1GRASS1", "1GRASS2", "1GRASS_L", "1GRASS_R", "1KERB_L", "1KERB_R"]
for name in potentially_to_delete:
    delete_object(name)
apply_steps("1ROAD1", "Asphalt", "Track")
#apply_steps("1GRASS0", "Grass", "Track")
#apply_steps("1GRASS1", "Grass", "Track")
apply_steps("1GRASS_L", "Grass", "Track") #smooth, external grass
apply_steps("1GRASS_R", "Grass", "Track") #smooth, external grass
apply_steps("1KERB_L", "Kerb", "Kerb")
apply_steps("1KERB_R", "Kerb", "Kerb")
#fill_inside_mesh("1GRASS1", "1GRASS2") #creates 1GRASS2

# save_as_fbx("./my_track.fbx")