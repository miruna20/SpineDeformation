import Sofa
import Sofa.Core
import SofaRuntime
import Sofa.Gui

import json

SofaRuntime.importPlugin('SofaComponentAll')
SofaRuntime.importPlugin("Sofa.Component.StateContainer")
SofaRuntime.importPlugin("SofaOpenglVisual")

USE_GUI = True

constant_force_fields_jane = {
    'v1': '0.1 0.0 0.0',
    'vert2': '0.1 0.1 0.0',
    'vert3': '0.2 0.1 0.0',
    'vert4': '0.1 0.1 0',
    'vert5': '0.1 0 0',

}
constant_force_fields_scaled = {
    'v1': '0.0 100 0.0',
    'vert2': '100 100 0.0',
    'vert3': '100 200 0.0',
    'vert4': '100 100 0',
    'vert5': '0 100 0',

}
constant_force_fields_ours = {
    'v1': '0.0 10 0.0',
    'vert2': '10 10 0.0',
    'vert3': '10 20 0.0',
    'vert4': '10 10 0',
    'vert5': '0 10 0',
}

def add_collision_function(rootNode):
    # Collision function
    rootNode.addObject('DefaultPipeline', verbose='0', name='CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='PenalityContactForceField', name='collision response')
    # rootNode.addObject('PenaltyContactForceField', response='default', name='collision response')

    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance='0.0005', contactDistance='0.000001',
                       angleCone='0.0')
    rootNode.addObject('DiscreteIntersection')

def add_vertebra_node(parent_node_vertebrae, nr_vertebra, spine_id):
    curr_vert_id = str(nr_vertebra + 1)
    curr_vert = parent_node_vertebrae.addChild('vert' + curr_vert_id)

    curr_vert.addObject('MechanicalObject', name='center_mass' + curr_vert_id, template='Rigid3d')
    curr_vert.addObject('RestShapeSpringsForceField', name='fixedPoints' + curr_vert_id, stiffness='5000',
                        angularStiffness='50')

    # mechanical model node as child from the current vertebra node
    mecha_node = curr_vert.addChild('mecha_node' + curr_vert_id)
    filename = 'mesh/spine' + str(spine_id) + 'v' + curr_vert_id + '.obj'

    mecha_node.addObject('MeshObjLoader',
                         name='ObjLoader' + curr_vert_id,
                         filename=filename,
                         printLog='true',
                         flipNormals='0')

    mecha_node.addObject('MeshTopology', name='v' + curr_vert_id + '_topology', src='@ObjLoader' + curr_vert_id)
    mecha_node.addObject('MechanicalObject', name='points' + curr_vert_id, template='Vec3d',
                         src='@v' + curr_vert_id + '_topology')
    mecha_node.addObject('TriangleCollisionModel')

    # visual model node  as a child of the mecha node
    visual_model = mecha_node.addChild('VisualModel')
    visual_model.addObject('OglModel', name='Visual', src='@../ObjLoader' + curr_vert_id, color='white',
                           translation='0 0 0')
    visual_model.addObject('IdentityMapping')

    # on each vertebra a different force is applied
    mecha_node.addObject('ConstantForceField', force=constant_force_fields_ours['vert' + curr_vert_id])

    mecha_node.addObject('RigidMapping', input='@..', output='@.')

    mecha_node.addObject('VTKExporter', filename='spine' + str(spine_id) + '_vert' + curr_vert_id + '_20_',
                         listening='true', edges='0', triangles='1', quads='0', tetras='0',
                         pointsDataFields='points' + curr_vert_id + '.position', exportEveryNumberOfSteps='20')

def add_springs_between_vertebrae(parent_node_vertebrae, nr_vertebra, springs_data):
    idx_first_vertebra = str(nr_vertebra)
    idx_second_vertebra = str(nr_vertebra + 1)
    curr_vertebrae_pair = 'v' + idx_first_vertebra + 'v' + idx_second_vertebra
    print('currvertpair' + str(curr_vertebrae_pair))

    object1 = '@vert' + idx_first_vertebra + '/mecha_node' + idx_first_vertebra + '/points' + idx_first_vertebra
    object2 = '@vert' + idx_second_vertebra + '/mecha_node' + idx_second_vertebra + '/points' + idx_second_vertebra

    # add springs between vertebrae bodies
    parent_node_vertebrae.addObject('StiffSpringForceField',
                       template='Vec3d',
                       name='box_springs' + idx_first_vertebra,
                       object1=object1,
                       object2=object2,
                       spring=springs_data['springs'][curr_vertebrae_pair]['body'])

    # add springs between facet left
    parent_node_vertebrae.addObject('StiffSpringForceField',
                       template='Vec3d',
                       name='facet_left_springs' + idx_first_vertebra,
                       object1=object1,
                       object2=object2,
                       spring=springs_data['springs'][curr_vertebrae_pair]['facet_left'])

    # add springs between facet right
    parent_node_vertebrae.addObject('StiffSpringForceField',
                       template='Vec3d',
                       name='facet_right_springs' + idx_first_vertebra,
                       object1=object1,
                       object2=object2,
                       spring=springs_data['springs'][curr_vertebrae_pair]['facet_right'])

def add_fixed_points(parent_node_vertebra, nr_vertebra, springs_data):
    fixed_points = parent_node_vertebra.addChild('fixed_points' + str(nr_vertebra))
    fixed_points.addObject('MechanicalObject',
                              name='Particles' + str(nr_vertebra),
                              template='Vec3d',
                              position=springs_data['fixed_points_positions']['v' + str(nr_vertebra)])
    fixed_points.addObject('MeshTopology',
                              name='Topology',
                              hexas=springs_data['fixed_points_indices']['v' + str(nr_vertebra)])
    fixed_points.addObject('UniformMass', name='Mass', vertexMass='1')
    fixed_points.addObject('FixedConstraint',
                              template='Vec3d',
                              name='fixedConstraint' + str(nr_vertebra),
                              indices=springs_data['fixed_points_indices']['v' + str(nr_vertebra)])

def createScene(rootNode):
    nr_vertebrae = 5
    spine_id = 'sub-verse500'
    path_json_file = '/home/miruna20/Documents/Thesis/Code/Preprocessing/master_thesis/samples/subverse500.json'

    # Visualization style
    rootNode.addObject('VisualStyle', displayFlags='showVisual showBehaviorModels showInteractionForceFields')

    add_collision_function(rootNode)


    # Parent node of all vertebrae
    together = rootNode.addChild('Together')
    together.addObject('EulerImplicitSolver', name='cg_odesolver', printLog='0', rayleighStiffness='0.09',
                       rayleighMass='0.1')
    together.addObject('CGLinearSolver', name='linear solver', iterations='1000', tolerance='1e-09', threshold='1e-15')

    # mech objects of vertebrae
    for i in range(nr_vertebrae):
        add_vertebra_node(parent_node_vertebrae=together,nr_vertebra=i,spine_id=spine_id)

    # read springs file
    file = open(path_json_file)
    json_data = json.load(file)

    # add springs in between vertebrae
    for i in range(1, nr_vertebrae):
        add_springs_between_vertebrae(parent_node_vertebrae=together, nr_vertebra=i,springs_data=json_data)


    # add positions of fixed points for v1
    add_fixed_points(parent_node_vertebra=together, nr_vertebra=1, springs_data=json_data)

    # add positions of fixed points for v5
    add_fixed_points(parent_node_vertebra=together, nr_vertebra=5, springs_data=json_data)


    # add springs between points in v1 and fixed points in v1 and the same for v5
    together.addObject('StiffSpringForceField',
                       name='points_fixed1',
                       object1='@fixed_points1/Particles1',
                       object2='@v1/mecha_node1/points1',
                       spring=json_data['springs']['v0v1'])

    together.addObject('StiffSpringForceField',
                       name='points_fixed5',
                       object1='@fixed_points5/Particles5',
                       object2='@vert5/mecha_node5/points5',
                       spring=json_data['springs']['v5v6'])

    return rootNode

def main():
    root = Sofa.Core.Node('root')
    createScene(root)
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(30):
            Sofa.Simulation.animate(root, root.dt.value)
            print(str(root.dt.value))
    else:
        # Find out the supported GUIs
        print("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
        # Launch the GUI (qt or qglviewer)
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        # Initialization of the scene will be done here
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()
        print("GUI was closed")

    print("Simulation is done.")

if __name__ == '__main__':
    main()


